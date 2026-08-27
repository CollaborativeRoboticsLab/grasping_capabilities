#pragma once

#include <any>
#include <chrono>
#include <string>
#include <vector>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{
namespace grasping_detail
{
/**
 * @brief Convert a vector-valued parameter into a string list.
 *
 * @param value Event parameter payload.
 * @return Parsed string list.
 */
inline std::vector<std::string> any_to_string_vector(const std::any & value)
{
  if (value.type() == typeid(std::vector<std::string>))
  {
    return std::any_cast<std::vector<std::string>>(value);
  }
  throw runner_exception("expected a string vector parameter");
}

/**
 * @brief Convert a vector-valued parameter into a numeric list.
 *
 * @param value Event parameter payload.
 * @return Parsed numeric list.
 */
inline std::vector<double> any_to_double_vector(const std::any & value)
{
  if (value.type() == typeid(std::vector<double>))
  {
    return std::any_cast<std::vector<double>>(value);
  }
  throw runner_exception("expected a double vector parameter");
}

/**
 * @brief Wait for a future in short steps until a deadline is reached.
 *
 * @tparam FutureT Future type with wait_for support.
 * @param future Future to wait on.
 * @param step Poll interval.
 * @param deadline Maximum completion time.
 * @return True when the future became ready before the deadline.
 */
template<typename FutureT>
inline bool wait_until_ready(
  FutureT & future,
  const std::chrono::milliseconds step,
  const std::chrono::steady_clock::time_point deadline)
{
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (future.wait_for(step) == std::future_status::ready)
    {
      return true;
    }
  }
  return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

/**
 * @brief Convert a MoveIt error code into a readable label.
 *
 * @param code Numeric MoveIt error code.
 * @return Human-readable code label.
 */
inline std::string moveit_error_to_string(const int32_t code)
{
  using moveit_msgs::msg::MoveItErrorCodes;
  switch (code)
  {
    case MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case MoveItErrorCodes::FAILURE:
      return "FAILURE";
    case MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case MoveItErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case MoveItErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case MoveItErrorCodes::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return "INVALID_GOAL_CONSTRAINTS";
    case MoveItErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case MoveItErrorCodes::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case MoveItErrorCodes::INVALID_OBJECT_NAME:
      return "INVALID_OBJECT_NAME";
    case MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return "COLLISION_CHECKING_UNAVAILABLE";
    case MoveItErrorCodes::ROBOT_STATE_STALE:
      return "ROBOT_STATE_STALE";
    case MoveItErrorCodes::SENSOR_INFO_STALE:
      return "SENSOR_INFO_STALE";
    case MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "UNKNOWN_ERROR_CODE(" + std::to_string(code) + ")";
  }
}

/**
 * @brief Build a MoveIt joint-constraint set from joint names and target positions.
 *
 * @param joint_names Ordered joint names.
 * @param joint_positions Ordered joint target positions.
 * @param tolerance Symmetric tolerance applied to each joint.
 * @return MoveIt constraints object.
 */
inline moveit_msgs::msg::Constraints build_joint_constraints(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  const double tolerance)
{
  moveit_msgs::msg::Constraints constraints;
  for (size_t index = 0; index < joint_names.size(); ++index)
  {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = joint_names[index];
    joint_constraint.position = joint_positions[index];
    joint_constraint.tolerance_above = tolerance;
    joint_constraint.tolerance_below = tolerance;
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);
  }
  return constraints;
}
}  // namespace grasping_detail
}  // namespace capabilities2_runner