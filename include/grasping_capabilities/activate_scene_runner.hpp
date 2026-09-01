#pragma once

#include <string>
#include <vector>

#include <grasping_msgs/action/activate_scene.hpp>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{
class ActivateSceneRunner : public ActionRunner<grasping_msgs::action::ActivateScene>
{
public:
  ActivateSceneRunner()
  : ActionRunner()
  {
  }

  void start(rclcpp::Node::SharedPtr node, const runner_opts & run_config, const std::string & bond_id) override
  {
    init_action(node, run_config, "/activate_scene", "grasping_msgs::action::ActivateScene");
    emit_started(bond_id, "", param_on_started());
  }

protected:
  grasping_msgs::action::ActivateScene::Goal generate_goal(
    capabilities2_events::EventParameters & parameters) override
  {
    grasping_msgs::action::ActivateScene::Goal goal_msg;
    goal_msg.reference.scene_name = std::any_cast<std::string>(
      parameters.get_value("scene_name", std::string("")));
    goal_msg.reference.package_name = std::any_cast<std::string>(
      parameters.get_value("package_name", std::string("")));
    goal_msg.reference.workspace_file = std::any_cast<std::string>(
      parameters.get_value("workspace_file", std::string("")));
    goal_msg.reference.scene_revision = std::any_cast<std::string>(
      parameters.get_value("scene_revision", std::string("")));
    goal_msg.force_reload = std::any_cast<bool>(parameters.get_value("force_reload", false));
    return goal_msg;
  }

  std::string generate_feedback(const grasping_msgs::action::ActivateScene::Feedback::ConstSharedPtr msg) override
  {
    if (msg->message.empty())
    {
      return msg->stage;
    }
    return msg->stage + ": " + msg->message;
  }

  void process_result(grasping_msgs::action::ActivateScene::Result::SharedPtr result) override
  {
    result_success_ = result->success;
    failure_reason_ = result->failure_reason;
    result_message_ = result->message;
    active_scene_name_ = result->active_scene.reference.scene_name;
    active_scene_package_name_ = result->active_scene.reference.package_name;
    active_scene_workspace_file_ = result->active_scene.reference.workspace_file;
    active_scene_revision_ = result->active_scene.reference.scene_revision;
    scene_handle_ = result->active_scene.scene_handle;
    workspace_base_frame_ = result->active_scene.workspace_base_frame;
    workspace_area_enabled_ = result->active_scene.workspace_area_enabled;
    object_names_ = result->active_scene.object_names;
  }

  capabilities2_events::EventParameters param_on_success() override
  {
    capabilities2_events::EventParameters parameters;
    parameters.set_value("success", result_success_, capabilities2_events::OptionType::BOOL);
    parameters.set_value("failure_reason", failure_reason_, capabilities2_events::OptionType::STRING);
    parameters.set_value("message", result_message_, capabilities2_events::OptionType::STRING);
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
    return param_on_success();
  }

private:
  bool result_success_{false};
  std::string failure_reason_;
  std::string result_message_;
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