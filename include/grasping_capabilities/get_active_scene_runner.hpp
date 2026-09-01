#pragma once

#include <string>
#include <vector>

#include <grasping_msgs/srv/get_active_scene.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>

namespace capabilities2_runner
{
class GetActiveSceneRunner : public ServiceRunner<grasping_msgs::srv::GetActiveScene>
{
public:
  GetActiveSceneRunner()
  : ServiceRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_service(node, run_config, "/get_active_scene", "grasping_msgs::srv::GetActiveScene");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::srv::GetActiveScene::Request generate_request(
    capabilities2_events::EventParameters & parameters) override
  {
    (void)parameters;
    return grasping_msgs::srv::GetActiveScene::Request();
  }

  void process_response(grasping_msgs::srv::GetActiveScene::Response::SharedPtr response) override
  {
    active_ = response->active;
    message_ = response->message;
    active_scene_name_ = response->active_scene.reference.scene_name;
    active_scene_package_name_ = response->active_scene.reference.package_name;
    active_scene_workspace_file_ = response->active_scene.reference.workspace_file;
    active_scene_revision_ = response->active_scene.reference.scene_revision;
    scene_handle_ = response->active_scene.scene_handle;
    workspace_base_frame_ = response->active_scene.workspace_base_frame;
    workspace_area_enabled_ = response->active_scene.workspace_area_enabled;
    object_names_ = response->active_scene.object_names;
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("active", active_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("message", message_, capabilities2_events::OptionType::STRING);
    parameters.set_value("active_scene_name", active_scene_name_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "active_scene_package_name", active_scene_package_name_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "active_scene_workspace_file", active_scene_workspace_file_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "active_scene_revision", active_scene_revision_, capabilities2_events::OptionType::STRING);
    parameters.set_value("scene_handle", scene_handle_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "workspace_base_frame", workspace_base_frame_, capabilities2_events::OptionType::STRING);
    parameters.set_value(
      "workspace_area_enabled", workspace_area_enabled_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("object_names", object_names_, capabilities2_events::OptionType::VECTOR_STRING);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value(
      "message", std::string("GetActiveScene service call failed."), capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  bool active_{false};
  std::string message_;
  std::string active_scene_name_;
  std::string active_scene_package_name_;
  std::string active_scene_workspace_file_;
  std::string active_scene_revision_;
  std::string scene_handle_;
  std::string workspace_base_frame_;
  bool workspace_area_enabled_{false};
  std::vector<std::string> object_names_;
};
}  // namespace capabilities2_runner