#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <grasping_msgs/srv/check_cartesian_pose_feasibility.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>

namespace capabilities2_runner
{
class CheckCartesianFeasibilityRunner
  : public ServiceRunner<grasping_msgs::srv::CheckCartesianPoseFeasibility>
{
public:
  CheckCartesianFeasibilityRunner()
  : ServiceRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_service(
      node, run_config, "/check_cartesian_pose_feasibility",
      "grasping_msgs::srv::CheckCartesianPoseFeasibility");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::srv::CheckCartesianPoseFeasibility::Request generate_request(
    capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::srv::CheckCartesianPoseFeasibility::Request request;
    request.pose.position.x = std::any_cast<double>(parameters.get_value("x", 0.0));
    request.pose.position.y = std::any_cast<double>(parameters.get_value("y", 0.0));
    request.pose.position.z = std::any_cast<double>(parameters.get_value("z", 0.0));
    request.pose.orientation.x = std::any_cast<double>(parameters.get_value("qx", 0.0));
    request.pose.orientation.y = std::any_cast<double>(parameters.get_value("qy", 0.0));
    request.pose.orientation.z = std::any_cast<double>(parameters.get_value("qz", 0.0));
    request.pose.orientation.w = std::any_cast<double>(parameters.get_value("qw", 1.0));
    request.planning_frame = std::any_cast<std::string>(
      parameters.get_value(
        "planning_frame",
        std::any_cast<std::string>(parameters.get_value("frame_id", std::string("")))));
    request.target_frame = std::any_cast<std::string>(parameters.get_value("target_frame", std::string("")));
    request.mode = std::any_cast<std::string>(parameters.get_value("mode", std::string("arm_only_plan")));
    return request;
  }

  void process_response(grasping_msgs::srv::CheckCartesianPoseFeasibility::Response::SharedPtr response) override
  {
    feasible_ = response->feasible;
    mode_used_ = response->mode_used;
    failure_reason_ = response->failure_reason;
    suggested_fallback_ = response->suggested_fallback;
    message_ = response->message;
    planning_frame_ = response->planning_frame;
    planning_pose_valid_ = response->planning_pose_valid;
    planning_pose_ = response->planning_pose;
    joint_solution_valid_ = response->joint_state_solution_valid;
    joint_names_ = response->joint_state_solution.name;
    joint_positions_.assign(
      response->joint_state_solution.position.begin(), response->joint_state_solution.position.end());
    collision_contacts_ = response->collision_contacts;
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("feasible", feasible_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("mode_used", mode_used_, capabilities2_events::OptionType::STRING);
    parameters.set_value("failure_reason", failure_reason_, capabilities2_events::OptionType::STRING);
    parameters.set_value("suggested_fallback", suggested_fallback_, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", message_, capabilities2_events::OptionType::STRING);
    parameters.set_value("planning_frame", planning_frame_, capabilities2_events::OptionType::STRING);
    parameters.set_value("planning_pose_valid", planning_pose_valid_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("planning_x", planning_pose_.position.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_y", planning_pose_.position.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_z", planning_pose_.position.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qx", planning_pose_.orientation.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qy", planning_pose_.orientation.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qz", planning_pose_.orientation.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("planning_qw", planning_pose_.orientation.w, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("joint_solution_valid", joint_solution_valid_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("joint_names", joint_names_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("joint_positions", joint_positions_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value("collision_contacts", collision_contacts_, capabilities2_events::OptionType::VECTOR_STRING);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", std::string("Cartesian feasibility service call failed."), capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  bool feasible_{false};
  std::string mode_used_;
  std::string failure_reason_;
  std::string suggested_fallback_;
  std::string message_;
  std::string planning_frame_;
  bool planning_pose_valid_{false};
  geometry_msgs::msg::Pose planning_pose_;
  bool joint_solution_valid_{false};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<std::string> collision_contacts_;
};
}  // namespace capabilities2_runner