#pragma once

#include <string>
#include <vector>

#include <grasping_msgs/srv/check_joint_pose_feasibility.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>

#include <grasping_capabilities/grasping_runner_utils.hpp>

namespace capabilities2_runner
{
class CheckJointFeasibilityRunner
  : public ServiceRunner<grasping_msgs::srv::CheckJointPoseFeasibility>
{
public:
  CheckJointFeasibilityRunner()
  : ServiceRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_service(
      node, run_config, "/check_joint_pose_feasibility",
      "grasping_msgs::srv::CheckJointPoseFeasibility");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::srv::CheckJointPoseFeasibility::Request generate_request(
    capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::srv::CheckJointPoseFeasibility::Request request;
    request.joint_names = grasping_detail::any_to_string_vector(
      parameters.get_value("joint_names", std::vector<std::string>{}));
    request.joint_positions = grasping_detail::any_to_double_vector(
      parameters.get_value("joint_positions", std::vector<double>{}));
    request.mode = std::any_cast<std::string>(parameters.get_value("mode", std::string("plan")));
    return request;
  }

  void process_response(grasping_msgs::srv::CheckJointPoseFeasibility::Response::SharedPtr response) override
  {
    feasible_ = response->feasible;
    mode_used_ = response->mode_used;
    failure_reason_ = response->failure_reason;
    suggested_fallback_ = response->suggested_fallback;
    message_ = response->message;
    planning_frame_ = response->planning_frame;
    checked_joint_state_valid_ = response->checked_joint_state_valid;
    checked_joint_names_ = response->checked_joint_state.name;
    checked_joint_positions_.assign(
      response->checked_joint_state.position.begin(), response->checked_joint_state.position.end());
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
    parameters.set_value("checked_joint_state_valid", checked_joint_state_valid_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("joint_names", checked_joint_names_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("joint_positions", checked_joint_positions_, capabilities2_events::OptionType::VECTOR_DOUBLE);
    parameters.set_value("collision_contacts", collision_contacts_, capabilities2_events::OptionType::VECTOR_STRING);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", std::string("Joint feasibility service call failed."), capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  bool feasible_{false};
  std::string mode_used_;
  std::string failure_reason_;
  std::string suggested_fallback_;
  std::string message_;
  std::string planning_frame_;
  bool checked_joint_state_valid_{false};
  std::vector<std::string> checked_joint_names_;
  std::vector<double> checked_joint_positions_;
  std::vector<std::string> collision_contacts_;
};
}  // namespace capabilities2_runner