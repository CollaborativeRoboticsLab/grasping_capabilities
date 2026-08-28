#pragma once

#include <string>
#include <vector>

#include <grasping_msgs/srv/list_named_poses.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>

namespace capabilities2_runner
{
class ListNamedPosesRunner : public ServiceRunner<grasping_msgs::srv::ListNamedPoses>
{
public:
  ListNamedPosesRunner()
  : ServiceRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_service(node, run_config, "/list_named_poses", "grasping_msgs::srv::ListNamedPoses");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::srv::ListNamedPoses::Request generate_request(
    capabilities2_events::EventParameters & parameters) override
  {
    (void)parameters;
    return grasping_msgs::srv::ListNamedPoses::Request();
  }

  void process_response(grasping_msgs::srv::ListNamedPoses::Response::SharedPtr response) override
  {
    pose_names_.clear();
    descriptions_.clear();
    for (const auto & descriptor : response->named_poses)
    {
      pose_names_.push_back(descriptor.pose_name);
      descriptions_.push_back(descriptor.description);
    }
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("pose_names", pose_names_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("descriptions", descriptions_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("pose_count", static_cast<int>(pose_names_.size()), capabilities2_events::OptionType::INT);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value(
      "message", std::string("Named pose listing service call failed."), capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  std::vector<std::string> pose_names_;
  std::vector<std::string> descriptions_;
};
}  // namespace capabilities2_runner