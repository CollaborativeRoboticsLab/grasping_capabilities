#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <grasping_msgs/srv/check_cartesian_pose_feasibility.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief Filter grasp candidates down to poses that are feasible for the manipulator.
 */
class FilterCartesianGraspsRunner : public ThreadTriggerRunner
{
public:
  FilterCartesianGraspsRunner()
  : ThreadTriggerRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_base(node, run_config);
    feasibility_client_ = node_->create_client<grasping_msgs::srv::CheckCartesianPoseFeasibility>(
      "/check_cartesian_pose_feasibility");
    emit_started(bond_id, "", param_on_started());
  }

  void stop(const std::string & bond_id, const std::string & instance_id = "") override
  {
    if (!node_ || !feasibility_client_)
    {
      throw runner_exception("cannot stop runner that was not started");
    }
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

protected:
  void execution(capabilities2_events::EventParameters parameters, const std::string & thread_id) override
  {
    const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    const std::string target_frame = std::any_cast<std::string>(
      parameters.get_value("target_frame", std::string("tcp")));
    const std::string default_planning_frame = std::any_cast<std::string>(
      parameters.get_value("planning_frame", std::string("")));
    const std::string mode = std::any_cast<std::string>(
      parameters.get_value("mode", std::string("arm_only_plan")));
    const double service_timeout_sec = std::any_cast<double>(
      parameters.get_value("service_timeout_sec", 10.0));

    frame_ids_ = std::any_cast<std::vector<std::string>>(
      parameters.get_value("frame_ids", std::vector<std::string>{}));
    positions_x_ = std::any_cast<std::vector<double>>(
      parameters.get_value("positions_x", std::vector<double>{}));
    positions_y_ = std::any_cast<std::vector<double>>(
      parameters.get_value("positions_y", std::vector<double>{}));
    positions_z_ = std::any_cast<std::vector<double>>(
      parameters.get_value("positions_z", std::vector<double>{}));
    orientations_x_ = std::any_cast<std::vector<double>>(
      parameters.get_value("orientations_x", std::vector<double>{}));
    orientations_y_ = std::any_cast<std::vector<double>>(
      parameters.get_value("orientations_y", std::vector<double>{}));
    orientations_z_ = std::any_cast<std::vector<double>>(
      parameters.get_value("orientations_z", std::vector<double>{}));
    orientations_w_ = std::any_cast<std::vector<double>>(
      parameters.get_value("orientations_w", std::vector<double>{}));

    clear_filtered_outputs();
    rejected_indices_.clear();

    if (!validate_inputs())
    {
      emit_failed(bond_id, instance_id, param_on_failure());
      return;
    }

    const auto service_timeout = std::chrono::duration<double>(service_timeout_sec);
    if (!feasibility_client_->wait_for_service(service_timeout))
    {
      status_message_ = "Cartesian feasibility service is unavailable.";
      emit_failed(bond_id, instance_id, param_on_failure());
      return;
    }

    for (std::size_t index = 0; index < frame_ids_.size(); ++index)
    {
      auto request = std::make_shared<grasping_msgs::srv::CheckCartesianPoseFeasibility::Request>();
      request->pose.position.x = positions_x_.at(index);
      request->pose.position.y = positions_y_.at(index);
      request->pose.position.z = positions_z_.at(index);
      request->pose.orientation.x = orientations_x_.at(index);
      request->pose.orientation.y = orientations_y_.at(index);
      request->pose.orientation.z = orientations_z_.at(index);
      request->pose.orientation.w = orientations_w_.at(index);
      request->planning_frame = default_planning_frame.empty() ? frame_ids_.at(index) : default_planning_frame;
      request->target_frame = target_frame;
      request->mode = mode;

      auto future = feasibility_client_->async_send_request(request);
      if (future.wait_for(service_timeout) != std::future_status::ready)
      {
        rejected_indices_.push_back(static_cast<int>(index));
        RCLCPP_WARN(
          node_->get_logger(),
          "FilterCartesianGraspsRunner timed out for grasp index=%zu frame_id='%s'",
          index,
          frame_ids_.at(index).c_str());
        continue;
      }

      const auto response = future.get();
      if (!response->feasible)
      {
        rejected_indices_.push_back(static_cast<int>(index));
        RCLCPP_INFO(
          node_->get_logger(),
          "FilterCartesianGraspsRunner rejected grasp index=%zu reason='%s' message='%s'",
          index,
          response->failure_reason.c_str(),
          response->message.c_str());
        continue;
      }

      filtered_frame_ids_.push_back(frame_ids_.at(index));
      filtered_positions_x_.push_back(positions_x_.at(index));
      filtered_positions_y_.push_back(positions_y_.at(index));
      filtered_positions_z_.push_back(positions_z_.at(index));
      filtered_orientations_x_.push_back(orientations_x_.at(index));
      filtered_orientations_y_.push_back(orientations_y_.at(index));
      filtered_orientations_z_.push_back(orientations_z_.at(index));
      filtered_orientations_w_.push_back(orientations_w_.at(index));
    }

    if (filtered_frame_ids_.empty())
    {
      status_message_ = "No reachable grasp poses remained after Cartesian feasibility filtering.";
      emit_failed(bond_id, instance_id, param_on_failure());
      return;
    }

    status_message_ = "Filtered " + std::to_string(frame_ids_.size()) +
      " grasps down to " + std::to_string(filtered_frame_ids_.size()) + " feasible candidates.";
    RCLCPP_INFO(node_->get_logger(), "%s", status_message_.c_str());
    emit_succeeded(bond_id, instance_id, param_on_success());
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value(
      "pose_count", static_cast<int>(filtered_frame_ids_.size()), capabilities2_events::OptionType::INT);
    parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
    parameters.set_value("frame_ids", filtered_frame_ids_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("positions_x", filtered_positions_x_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value("positions_y", filtered_positions_y_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value("positions_z", filtered_positions_z_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value(
      "orientations_x", filtered_orientations_x_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value(
      "orientations_y", filtered_orientations_y_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value(
      "orientations_z", filtered_orientations_z_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value(
      "orientations_w", filtered_orientations_w_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "pose_count", static_cast<int>(filtered_frame_ids_.size()), capabilities2_events::OptionType::INT);
    return parameters;
  }

private:
  bool validate_inputs()
  {
    const std::size_t pose_count = frame_ids_.size();
    if (pose_count == 0U)
    {
      status_message_ = "No grasp poses available to filter.";
      return false;
    }

    const bool arrays_match =
      positions_x_.size() == pose_count &&
      positions_y_.size() == pose_count &&
      positions_z_.size() == pose_count &&
      orientations_x_.size() == pose_count &&
      orientations_y_.size() == pose_count &&
      orientations_z_.size() == pose_count &&
      orientations_w_.size() == pose_count;

    if (!arrays_match)
    {
      status_message_ = "Grasp candidate arrays are inconsistent in length.";
      return false;
    }

    return true;
  }

  void clear_filtered_outputs()
  {
    filtered_frame_ids_.clear();
    filtered_positions_x_.clear();
    filtered_positions_y_.clear();
    filtered_positions_z_.clear();
    filtered_orientations_x_.clear();
    filtered_orientations_y_.clear();
    filtered_orientations_z_.clear();
    filtered_orientations_w_.clear();
  }

  rclcpp::Client<grasping_msgs::srv::CheckCartesianPoseFeasibility>::SharedPtr feasibility_client_;
  std::vector<std::string> frame_ids_;
  std::vector<double> positions_x_;
  std::vector<double> positions_y_;
  std::vector<double> positions_z_;
  std::vector<double> orientations_x_;
  std::vector<double> orientations_y_;
  std::vector<double> orientations_z_;
  std::vector<double> orientations_w_;
  std::vector<std::string> filtered_frame_ids_;
  std::vector<double> filtered_positions_x_;
  std::vector<double> filtered_positions_y_;
  std::vector<double> filtered_positions_z_;
  std::vector<double> filtered_orientations_x_;
  std::vector<double> filtered_orientations_y_;
  std::vector<double> filtered_orientations_z_;
  std::vector<double> filtered_orientations_w_;
  std::vector<int> rejected_indices_;
  std::string status_message_;
};
}  // namespace capabilities2_runner