#pragma once

#include <string>

#include <grasping_msgs/action/move_to_pose.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief Send Cartesian pose goals through the grasping MoveToPose action.
 */
class MoveToCartesianPoseRunner : public ActionRunner<grasping_msgs::action::MoveToPose>
{
public:
  /**
   * @brief Default constructor required for pluginlib.
   */
  MoveToCartesianPoseRunner()
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
    init_action(node, run_config, "/move_arm_to_pose", "grasping_msgs::action::MoveToPose");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief Convert event parameters into a MoveToPose goal.
   *
   * @param parameters Event parameters containing pose fields.
   * @return Cartesian pose goal.
   */
  grasping_msgs::action::MoveToPose::Goal generate_goal(capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::action::MoveToPose::Goal goal_msg;
    goal_msg.target_pose.header.frame_id = std::any_cast<std::string>(
      parameters.get_value("frame_id", std::string("base_link")));
    goal_msg.target_pose.pose.position.x = std::any_cast<double>(parameters.get_value("x", 0.0));
    goal_msg.target_pose.pose.position.y = std::any_cast<double>(parameters.get_value("y", 0.0));
    goal_msg.target_pose.pose.position.z = std::any_cast<double>(parameters.get_value("z", 0.0));
    goal_msg.target_pose.pose.orientation.x = std::any_cast<double>(parameters.get_value("qx", 0.0));
    goal_msg.target_pose.pose.orientation.y = std::any_cast<double>(parameters.get_value("qy", 0.0));
    goal_msg.target_pose.pose.orientation.z = std::any_cast<double>(parameters.get_value("qz", 0.0));
    goal_msg.target_pose.pose.orientation.w = std::any_cast<double>(parameters.get_value("qw", 1.0));
    return goal_msg;
  }

  /**
   * @brief Forward the grasping action feedback state.
   *
   * @param msg Feedback message from the action server.
   * @return Human-readable feedback text.
   */
  std::string generate_feedback(const grasping_msgs::action::MoveToPose::Feedback::ConstSharedPtr msg) override
  {
    return msg->state;
  }

  /**
   * @brief Cache the action result message for later event emission.
   *
   * @param result Action result.
   */
  void process_result(grasping_msgs::action::MoveToPose::Result::SharedPtr result) override
  {
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
  std::string result_message_;
};
}  // namespace capabilities2_runner