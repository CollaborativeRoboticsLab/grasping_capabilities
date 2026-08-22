#pragma once

#include <chrono>
#include <string>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/threadtrigger_runner.hpp>

#include <moveit2_capabilities/moveit2_runner_utils.hpp>

namespace capabilities2_runner
{
/**
 * @brief Build and send direct MoveIt joint-space goals.
 */
class MoveToJointPoseRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Default constructor required for pluginlib.
   */
  MoveToJointPoseRunner()
  : ThreadTriggerRunner()
  {
  }

  /**
   * @brief Initialize the MoveGroup action client and emit the started event.
   *
   * @param node Shared ROS node.
   * @param run_config Runner configuration.
   * @param bond_id Runner bond identifier.
   */
  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_base(node, run_config);

    std::string action_name = "/move_action";
    try
    {
      action_name = get_action_name_by_type("moveit_msgs::action::MoveGroup");
    }
    catch (const runner_exception &)
    {
    }

    action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, action_name);
    if (!action_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      throw runner_exception("failed to connect to MoveGroup action server: " + action_name);
    }

    emit_started(bond_id, "", param_on_started());
  }

  /**
   * @brief Emit the stopped event for this runner.
   *
   * @param bond_id Runner bond identifier.
   * @param instance_id Optional capability instance identifier.
   */
  void stop(const std::string & bond_id, const std::string & instance_id = "") override
  {
    if (!node_ || !action_client_)
    {
      throw runner_exception("cannot stop runner that was not started");
    }
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

protected:
  /**
   * @brief Convert joint target parameters into a direct MoveGroup request.
   *
   * @param parameters Event parameters for the joint goal.
   * @param thread_id Thread identifier for the running trigger instance.
   */
  void execution(capabilities2_events::EventParameters parameters, const std::string & thread_id) override
  {
    const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    try
    {
      const auto joint_names = moveit2_detail::any_to_string_vector(
        parameters.get_value("joint_names_csv", std::string("")));
      const auto joint_positions = moveit2_detail::any_to_double_vector(
        parameters.get_value("joint_positions_csv", std::string("")));

      if (joint_names.empty())
      {
        throw runner_exception("joint_names_csv must contain at least one joint name");
      }
      if (joint_names.size() != joint_positions.size())
      {
        throw runner_exception("joint_names_csv and joint_positions_csv must contain the same number of values");
      }

      moveit_msgs::action::MoveGroup::Goal goal_msg;
      goal_msg.request.group_name = std::any_cast<std::string>(
        parameters.get_value("planning_group", std::string("manipulator")));
      goal_msg.request.allowed_planning_time = std::any_cast<double>(
        parameters.get_value("allowed_planning_time", 5.0));
      goal_msg.request.num_planning_attempts = std::any_cast<int>(
        parameters.get_value("num_planning_attempts", 5));
      goal_msg.request.max_velocity_scaling_factor = std::any_cast<double>(
        parameters.get_value("max_velocity_scaling", 0.2));
      goal_msg.request.max_acceleration_scaling_factor = std::any_cast<double>(
        parameters.get_value("max_acceleration_scaling", 0.2));

      const auto planning_pipeline_id = std::any_cast<std::string>(
        parameters.get_value("planning_pipeline_id", std::string("")));
      const auto planner_id = std::any_cast<std::string>(
        parameters.get_value("planner_id", std::string("")));
      if (!planning_pipeline_id.empty())
      {
        goal_msg.request.pipeline_id = planning_pipeline_id;
      }
      if (!planner_id.empty())
      {
        goal_msg.request.planner_id = planner_id;
      }

      const double joint_goal_tolerance = std::any_cast<double>(
        parameters.get_value("joint_goal_tolerance_rad", 0.001));
      goal_msg.request.goal_constraints = {
        moveit2_detail::build_joint_constraints(joint_names, joint_positions, joint_goal_tolerance)};
      goal_msg.planning_options.plan_only = false;
      goal_msg.planning_options.look_around = false;
      goal_msg.planning_options.replan = false;
      goal_msg.planning_options.replan_attempts = 0;

      auto send_future = action_client_->async_send_goal(goal_msg);
      const auto send_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      if (!moveit2_detail::wait_until_ready(send_future, std::chrono::milliseconds(100), send_deadline))
      {
        throw runner_exception("timed out while sending MoveGroup goal");
      }

      const auto goal_handle = send_future.get();
      if (!goal_handle)
      {
        throw runner_exception("MoveGroup goal was rejected");
      }

      auto result_future = action_client_->async_get_result(goal_handle);
      const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
      if (!moveit2_detail::wait_until_ready(result_future, std::chrono::milliseconds(100), result_deadline))
      {
        throw runner_exception("timed out while waiting for MoveGroup result");
      }

      const auto wrapped_result = result_future.get();
      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        throw runner_exception("MoveGroup action did not finish successfully");
      }
      if (!wrapped_result.result)
      {
        throw runner_exception("MoveGroup returned an empty result");
      }
      if (wrapped_result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        throw runner_exception(
          "MoveGroup failed with " + moveit2_detail::moveit_error_to_string(wrapped_result.result->error_code.val));
      }

      status_message_ = "Joint-space motion completed successfully.";
      emit_succeeded(bond_id, instance_id, param_on_success());
    }
    catch (const std::exception & error)
    {
      status_message_ = error.what();
      RCLCPP_ERROR(node_->get_logger(), "%s", status_message_.c_str());
      emit_failed(bond_id, instance_id, param_on_failure());
    }
  }

  /**
   * @brief Emit the final motion status message on success.
   *
   * @return Event parameters for the success event.
   */
  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
    return parameters;
  }

  /**
   * @brief Emit the final motion status message on failure.
   *
   * @return Event parameters for the failure event.
   */
  capabilities2_events::EventParameters param_on_failure() override
  {
    return param_on_success();
  }

private:
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
  std::string status_message_;
};
}  // namespace capabilities2_runner