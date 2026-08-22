#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief Resolve the current manipulator pose from TF.
 */
class CurrentCartesianPoseRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Default constructor required for pluginlib.
   */
  CurrentCartesianPoseRunner()
  : ThreadTriggerRunner()
  {
  }

  /**
   * @brief Initialize TF resources and emit the started event.
   *
   * @param node Shared ROS node.
   * @param run_config Runner configuration.
   * @param bond_id Runner bond identifier.
   */
  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_base(node, run_config);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    if (!node_ || !tf_listener_)
    {
      throw runner_exception("cannot stop runner that was not started");
    }
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

protected:
  /**
   * @brief Look up the requested transform and emit the current pose.
   *
   * @param parameters Event parameters describing the source and target frames.
   * @param thread_id Thread identifier for the running trigger instance.
   */
  void execution(capabilities2_events::EventParameters parameters, const std::string & thread_id) override
  {
    const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    const std::string from_frame = std::any_cast<std::string>(
      parameters.get_value("from_frame", std::string("base_link")));
    const std::string to_frame = std::any_cast<std::string>(
      parameters.get_value("to_frame", std::string("tcp")));
    const double timeout_sec = std::any_cast<double>(parameters.get_value("time_buffer", 5.0));

    try
    {
      transform_ = tf_buffer_->lookupTransform(
        from_frame,
        to_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout_sec));

      tf2::Quaternion quaternion(
        transform_.transform.rotation.x,
        transform_.transform.rotation.y,
        transform_.transform.rotation.z,
        transform_.transform.rotation.w);
      tf2::Matrix3x3 matrix(quaternion);
      matrix.getRPY(roll_, pitch_, yaw_);

      emit_succeeded(bond_id, instance_id, param_on_success());
    }
    catch (const tf2::TransformException & error)
    {
      status_message_ = error.what();
      RCLCPP_ERROR(node_->get_logger(), "%s", status_message_.c_str());
      emit_failed(bond_id, instance_id, param_on_failure());
    }
  }

  /**
   * @brief Emit the resolved Cartesian pose.
   *
   * @return Event parameters for the success event.
   */
  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("x", transform_.transform.translation.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("y", transform_.transform.translation.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("z", transform_.transform.translation.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("qx", transform_.transform.rotation.x, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("qy", transform_.transform.rotation.y, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("qz", transform_.transform.rotation.z, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("qw", transform_.transform.rotation.w, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("roll", roll_, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("pitch", pitch_, capabilities2_events::OptionType::DOUBLE);
    parameters.set_value("yaw", yaw_, capabilities2_events::OptionType::DOUBLE);
    return parameters;
  }

  /**
   * @brief Emit the TF lookup error on failure.
   *
   * @return Event parameters for the failure event.
   */
  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped transform_;
  std::string status_message_;
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
};
}  // namespace capabilities2_runner