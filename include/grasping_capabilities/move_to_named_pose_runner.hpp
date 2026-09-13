#pragma once

#include <string>

#include <grasping_msgs/action/move_to_named_pose.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief Send configured named-pose goals through the grasping MoveToNamedPose action.
 */
class MoveToNamedPoseRunner : public ActionRunner<grasping_msgs::action::MoveToNamedPose>
{
public:
  /**
   * @brief Default constructor required for pluginlib.
   */
  MoveToNamedPoseRunner()
  : ActionRunner()
  {
  }

  /**
   * @brief Initialize the action client and emit the started event.
   *
   * @param node Shared ROS node.
   * @param run_config Runner configuration.
   * @param bond_id Runner bond identifier.
   */
  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_action(node, run_config, "/move_arm_to_named_pose", "grasping_msgs::action::MoveToNamedPose");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief Convert event parameters into a MoveToNamedPose goal.
   *
   * @param parameters Event parameters containing the pose name.
   * @return Named-pose goal.
   */
  grasping_msgs::action::MoveToNamedPose::Goal generate_goal(capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::action::MoveToNamedPose::Goal goal_msg;
    goal_msg.pose_name = std::any_cast<std::string>(parameters.get_value("pose_name", std::string("")));
    return goal_msg;
  }

  /**
   * @brief Forward the grasping action feedback state.
   *
   * @param msg Feedback message from the action server.
   * @return Human-readable feedback text.
   */
  std::string generate_feedback(const grasping_msgs::action::MoveToNamedPose::Feedback::ConstSharedPtr msg) override
  {
    return msg->state;
  }

  std::string result_diagnostic() const override
  {
    return result_message_;
  }

  /**
   * @brief Cache the action result message for later event emission.
   *
   * @param result Action result.
   */
  void process_result(grasping_msgs::action::MoveToNamedPose::Result::SharedPtr result) override
  {
    result_success_ = result->success;
    result_message_ = result->message;
  }

  /**
   * @brief Emit the final action status message on success.
   *
   * @return Event parameters for the success event.
   */
  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("success", result_success_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("message", result_message_, capabilities2_events::OptionType::STRING);
    return parameters;
  }

  /**
   * @brief Reuse the final action status message on failure.
   *
   * @return Event parameters for the failure event.
   */
  capabilities2_events::EventParameters param_on_failure() override
  {
    return param_on_success();
  }

private:
  bool result_success_{false};
  std::string result_message_;
};
}  // namespace capabilities2_runner