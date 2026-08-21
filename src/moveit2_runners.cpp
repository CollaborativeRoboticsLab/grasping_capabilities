#include <pluginlib/class_list_macros.hpp>

#include <capabilities2_runner/runner_base.hpp>
#include <moveit2_capabilities/moveit2_runners.hpp>

PLUGINLIB_EXPORT_CLASS(capabilities2_runner::MoveToCartesianPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::MoveToJointPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::MoveToNamedPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::CurrentCartesianPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::CurrentJointPoseRunner, capabilities2_runner::RunnerBase)