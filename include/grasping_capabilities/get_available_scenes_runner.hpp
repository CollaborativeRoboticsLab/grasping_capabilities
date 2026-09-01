#pragma once

#include <string>
#include <vector>

#include <grasping_msgs/srv/list_available_scenes.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/service_runner.hpp>

namespace capabilities2_runner
{
class GetAvailableScenesRunner : public ServiceRunner<grasping_msgs::srv::ListAvailableScenes>
{
public:
  GetAvailableScenesRunner()
  : ServiceRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_service(node, run_config, "/list_available_scenes", "grasping_msgs::srv::ListAvailableScenes");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::srv::ListAvailableScenes::Request generate_request(
    capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::srv::ListAvailableScenes::Request request;
    request.requested_mode = std::any_cast<std::string>(
      parameters.get_value("requested_mode", std::string("")));
    request.required_tags = std::any_cast<std::vector<std::string>>(
      parameters.get_value("required_tags", std::vector<std::string>{}));
    return request;
  }

  void process_response(grasping_msgs::srv::ListAvailableScenes::Response::SharedPtr response) override
  {
    message_ = response->message;
    scene_names_.clear();
    package_names_.clear();
    workspace_files_.clear();
    scene_revisions_.clear();
    station_frames_.clear();
    selector_profiles_.clear();
    descriptions_.clear();
    for (const auto & scene : response->scenes)
    {
      scene_names_.push_back(scene.reference.scene_name);
      package_names_.push_back(scene.reference.package_name);
      workspace_files_.push_back(scene.reference.workspace_file);
      scene_revisions_.push_back(scene.reference.scene_revision);
      station_frames_.push_back(scene.station_frame);
      selector_profiles_.push_back(scene.selector_profile);
      descriptions_.push_back(scene.description);
    }
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("message", message_, capabilities2_events::OptionType::STRING);
    parameters.set_value("scene_names", scene_names_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("package_names", package_names_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value(
      "workspace_files", workspace_files_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value(
      "scene_revisions", scene_revisions_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value(
      "station_frames", station_frames_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value(
      "selector_profiles", selector_profiles_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("descriptions", descriptions_, capabilities2_events::OptionType::VECTOR_STRING);
    parameters.set_value("scene_count", static_cast<int>(scene_names_.size()), capabilities2_events::OptionType::INT);
    return parameters;
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value(
      "message", std::string("ListAvailableScenes service call failed."), capabilities2_events::OptionType::STRING);
    return parameters;
  }

private:
  std::string message_;
  std::vector<std::string> scene_names_;
  std::vector<std::string> package_names_;
  std::vector<std::string> workspace_files_;
  std::vector<std::string> scene_revisions_;
  std::vector<std::string> station_frames_;
  std::vector<std::string> selector_profiles_;
  std::vector<std::string> descriptions_;
};
}  // namespace capabilities2_runner