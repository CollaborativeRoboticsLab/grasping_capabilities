#pragma once

#include <string>

#include <grasping_msgs/action/move_to_joint_pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>

#include <moveit2_capabilities/moveit2_runner_utils.hpp>

namespace capabilities2_runner
{
/**
 * @brief Send joint-space goals through the grasping MoveToJointPose action.
 */
class MoveToJointPoseRunner : public ActionRunner<grasping_msgs::action::MoveToJointPose>
{
public:
  /**
   * @brief Default constructor required for pluginlib.
   */
  MoveToJointPoseRunner()
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
    init_action(node, run_config, "/move_arm_to_joint_pose", "grasping_msgs::action::MoveToJointPose");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief Convert joint target parameters into a MoveToJointPose goal.
   *
   * @param parameters Event parameters for the joint goal.
   * @return Joint-space goal.
   */
  grasping_msgs::action::MoveToJointPose::Goal generate_goal(
    capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::action::MoveToJointPose::Goal goal_msg;

    const auto joint_names = moveit2_detail::any_to_string_vector(
      parameters.get_value("joint_names", std::vector<std::string>{}));
    const auto joint_positions = moveit2_detail::any_to_double_vector(
      parameters.get_value("joint_positions", std::vector<double>{}));

    if (joint_names.empty())
    {
      throw runner_exception("joint_names must contain at least one joint name");
    }
    if (joint_names.size() != joint_positions.size())
    {
      throw runner_exception("joint_names and joint_positions must contain the same number of values");
    }

    goal_msg.target_joint_state = sensor_msgs::msg::JointState();
    goal_msg.target_joint_state.name = joint_names;
    goal_msg.target_joint_state.position = joint_positions;
    return goal_msg;
  }

  /**
   * @brief Forward the grasping action feedback state.
   *
   * @param msg Feedback message from the action server.
   * @return Human-readable feedback text.
   */
  std::string generate_feedback(
    const grasping_msgs::action::MoveToJointPose::Feedback::ConstSharedPtr msg) override
  {
    return msg->state;
  }

  /**
   * @brief Cache the action result message for later event emission.
   *
   * @param result Action result.
   */
  void process_result(grasping_msgs::action::MoveToJointPose::Result::SharedPtr result) override
  {
    result_message_ = result->message;
  }

  /**
   * @brief Emit the final motion status message on success.
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
   * @brief Emit the final motion status message on failure.
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