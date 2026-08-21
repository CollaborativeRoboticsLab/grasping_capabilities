#pragma once

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grasping_msgs/action/move_to_named_pose.hpp>
#include <grasping_msgs/action/move_to_pose.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <capabilities2_events/event_parameters.hpp>
#include <capabilities2_runner/action_runner.hpp>
#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{
    namespace moveit2_detail
    {
        inline std::string trim(const std::string &value)
        {
            const auto begin = value.find_first_not_of(" \t\n\r");
            if (begin == std::string::npos)
            {
                return "";
            }
            const auto end = value.find_last_not_of(" \t\n\r");
            return value.substr(begin, end - begin + 1);
        }

        inline std::vector<std::string> split_csv(const std::string &text)
        {
            std::vector<std::string> values;
            std::stringstream stream(text);
            std::string item;
            while (std::getline(stream, item, ','))
            {
                item = trim(item);
                if (!item.empty())
                {
                    values.push_back(item);
                }
            }
            return values;
        }

        inline std::string join_csv(const std::vector<std::string> &values)
        {
            std::ostringstream stream;
            for (size_t index = 0; index < values.size(); ++index)
            {
                if (index > 0)
                {
                    stream << ",";
                }
                stream << values[index];
            }
            return stream.str();
        }

        inline std::string join_csv(const std::vector<double> &values)
        {
            std::ostringstream stream;
            stream.setf(std::ios::fixed);
            stream.precision(6);
            for (size_t index = 0; index < values.size(); ++index)
            {
                if (index > 0)
                {
                    stream << ",";
                }
                stream << values[index];
            }
            return stream.str();
        }

        inline std::vector<std::string> any_to_string_vector(const std::any &value)
        {
            if (value.type() == typeid(std::string))
            {
                return split_csv(std::any_cast<std::string>(value));
            }
            if (value.type() == typeid(std::vector<std::string>))
            {
                return std::any_cast<std::vector<std::string>>(value);
            }
            throw runner_exception("expected a string or string vector parameter");
        }

        inline std::vector<double> any_to_double_vector(const std::any &value)
        {
            if (value.type() == typeid(std::string))
            {
                std::vector<double> numbers;
                for (const auto &item : split_csv(std::any_cast<std::string>(value)))
                {
                    numbers.push_back(std::stod(item));
                }
                return numbers;
            }
            if (value.type() == typeid(std::vector<double>))
            {
                return std::any_cast<std::vector<double>>(value);
            }
            throw runner_exception("expected a string or double vector parameter");
        }

        inline bool wait_for_future(const std::shared_future<typename rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr> &future,
                                    const std::chrono::milliseconds timeout)
        {
            return future.wait_for(timeout) == std::future_status::ready;
        }

        template <typename FutureT>
        inline bool wait_until_ready(FutureT &future, const std::chrono::milliseconds step,
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

        inline moveit_msgs::msg::Constraints build_joint_constraints(
            const std::vector<std::string> &joint_names,
            const std::vector<double> &joint_positions,
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
    } // namespace moveit2_detail

    class MoveToCartesianPoseRunner : public ActionRunner<grasping_msgs::action::MoveToPose>
    {
    public:
        MoveToCartesianPoseRunner() : ActionRunner()
        {
        }

        void start(rclcpp::Node::SharedPtr node, const runner_opts &run_config, const std::string &bond_id) override
        {
            init_action(node, run_config, "/move_arm_to_pose", "grasping_msgs::action::MoveToPose");
            emit_started(bond_id, "", param_on_started());
        }

    protected:
        grasping_msgs::action::MoveToPose::Goal generate_goal(capabilities2_events::EventParameters &parameters) override
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

        std::string generate_feedback(const grasping_msgs::action::MoveToPose::Feedback::ConstSharedPtr msg) override
        {
            return msg->state;
        }

        void process_result(grasping_msgs::action::MoveToPose::Result::SharedPtr result) override
        {
            result_message_ = result->message;
        }

        capabilities2_events::EventParameters param_on_success() override
        {
            capabilities2_events::EventParameters parameters;
            parameters.set_value("message", result_message_, capabilities2_events::OptionType::STRING);
            return parameters;
        }

        capabilities2_events::EventParameters param_on_failure() override
        {
            return param_on_success();
        }

    private:
        std::string result_message_;
    };

    class MoveToNamedPoseRunner : public ActionRunner<grasping_msgs::action::MoveToNamedPose>
    {
    public:
        MoveToNamedPoseRunner() : ActionRunner()
        {
        }

        void start(rclcpp::Node::SharedPtr node, const runner_opts &run_config, const std::string &bond_id) override
        {
            init_action(node, run_config, "/move_arm_to_named_pose", "grasping_msgs::action::MoveToNamedPose");
            emit_started(bond_id, "", param_on_started());
        }

    protected:
        grasping_msgs::action::MoveToNamedPose::Goal generate_goal(capabilities2_events::EventParameters &parameters) override
        {
            grasping_msgs::action::MoveToNamedPose::Goal goal_msg;
            goal_msg.pose_name = std::any_cast<std::string>(parameters.get_value("pose_name", std::string("")));
            return goal_msg;
        }

        std::string generate_feedback(const grasping_msgs::action::MoveToNamedPose::Feedback::ConstSharedPtr msg) override
        {
            return msg->state;
        }

        void process_result(grasping_msgs::action::MoveToNamedPose::Result::SharedPtr result) override
        {
            result_message_ = result->message;
        }

        capabilities2_events::EventParameters param_on_success() override
        {
            capabilities2_events::EventParameters parameters;
            parameters.set_value("message", result_message_, capabilities2_events::OptionType::STRING);
            return parameters;
        }

        capabilities2_events::EventParameters param_on_failure() override
        {
            return param_on_success();
        }

    private:
        std::string result_message_;
    };

    class MoveToJointPoseRunner : public ThreadTriggerRunner
    {
    public:
        MoveToJointPoseRunner() : ThreadTriggerRunner()
        {
        }

        void start(rclcpp::Node::SharedPtr node, const runner_opts &run_config, const std::string &bond_id) override
        {
            init_base(node, run_config);

            std::string action_name = "/move_action";
            try
            {
                action_name = get_action_name_by_type("moveit_msgs::action::MoveGroup");
            }
            catch (const runner_exception &)
            {
            }

            action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, action_name);
            if (!action_client_->wait_for_action_server(std::chrono::seconds(30)))
            {
                throw runner_exception("failed to connect to MoveGroup action server: " + action_name);
            }

            emit_started(bond_id, "", param_on_started());
        }

        void stop(const std::string &bond_id, const std::string &instance_id = "") override
        {
            if (!node_ || !action_client_)
            {
                throw runner_exception("cannot stop runner that was not started");
            }
            emit_stopped(bond_id, instance_id, param_on_stopped());
        }

    protected:
        void execution(capabilities2_events::EventParameters parameters, const std::string &thread_id) override
        {
            const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
            const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

            try
            {
                const auto joint_names = moveit2_detail::any_to_string_vector(
                    parameters.get_value("joint_names_csv", std::string("")));
                const auto joint_positions = moveit2_detail::any_to_double_vector(
                    parameters.get_value("joint_positions_csv", std::string("")));

                if (joint_names.empty())
                {
                    throw runner_exception("joint_names_csv must contain at least one joint name");
                }
                if (joint_names.size() != joint_positions.size())
                {
                    throw runner_exception("joint_names_csv and joint_positions_csv must contain the same number of values");
                }

                moveit_msgs::action::MoveGroup::Goal goal_msg;
                goal_msg.request.group_name = std::any_cast<std::string>(
                    parameters.get_value("planning_group", std::string("manipulator")));
                goal_msg.request.allowed_planning_time = std::any_cast<double>(
                    parameters.get_value("allowed_planning_time", 5.0));
                goal_msg.request.num_planning_attempts = std::any_cast<int>(
                    parameters.get_value("num_planning_attempts", 5));
                goal_msg.request.max_velocity_scaling_factor = std::any_cast<double>(
                    parameters.get_value("max_velocity_scaling", 0.2));
                goal_msg.request.max_acceleration_scaling_factor = std::any_cast<double>(
                    parameters.get_value("max_acceleration_scaling", 0.2));

                const auto planning_pipeline_id = std::any_cast<std::string>(
                    parameters.get_value("planning_pipeline_id", std::string("")));
                const auto planner_id = std::any_cast<std::string>(
                    parameters.get_value("planner_id", std::string("")));
                if (!planning_pipeline_id.empty())
                {
                    goal_msg.request.pipeline_id = planning_pipeline_id;
                }
                if (!planner_id.empty())
                {
                    goal_msg.request.planner_id = planner_id;
                }

                const double joint_goal_tolerance = std::any_cast<double>(
                    parameters.get_value("joint_goal_tolerance_rad", 0.001));
                goal_msg.request.goal_constraints = {
                    moveit2_detail::build_joint_constraints(joint_names, joint_positions, joint_goal_tolerance)};
                goal_msg.planning_options.plan_only = false;
                goal_msg.planning_options.look_around = false;
                goal_msg.planning_options.replan = false;
                goal_msg.planning_options.replan_attempts = 0;

                auto send_future = action_client_->async_send_goal(goal_msg);
                const auto send_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
                if (!moveit2_detail::wait_until_ready(send_future, std::chrono::milliseconds(100), send_deadline))
                {
                    throw runner_exception("timed out while sending MoveGroup goal");
                }

                const auto goal_handle = send_future.get();
                if (!goal_handle || !goal_handle->is_result_aware())
                {
                    throw runner_exception("MoveGroup goal was rejected");
                }

                auto result_future = action_client_->async_get_result(goal_handle);
                const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
                if (!moveit2_detail::wait_until_ready(result_future, std::chrono::milliseconds(100), result_deadline))
                {
                    throw runner_exception("timed out while waiting for MoveGroup result");
                }

                const auto wrapped_result = result_future.get();
                if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
                {
                    throw runner_exception("MoveGroup action did not finish successfully");
                }
                if (!wrapped_result.result)
                {
                    throw runner_exception("MoveGroup returned an empty result");
                }
                if (wrapped_result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                {
                    throw runner_exception(
                        "MoveGroup failed with " + moveit2_detail::moveit_error_to_string(wrapped_result.result->error_code.val));
                }

                status_message_ = "Joint-space motion completed successfully.";
                emit_succeeded(bond_id, instance_id, param_on_success());
            }
            catch (const std::exception &error)
            {
                status_message_ = error.what();
                RCLCPP_ERROR(node_->get_logger(), "%s", status_message_.c_str());
                emit_failed(bond_id, instance_id, param_on_failure());
            }
        }

        capabilities2_events::EventParameters param_on_success() override
        {
            capabilities2_events::EventParameters parameters;
            parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
            return parameters;
        }

        capabilities2_events::EventParameters param_on_failure() override
        {
            return param_on_success();
        }

    private:
        rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
        std::string status_message_;
    };

    class CurrentCartesianPoseRunner : public ThreadTriggerRunner
    {
    public:
        CurrentCartesianPoseRunner() : ThreadTriggerRunner()
        {
        }

        void start(rclcpp::Node::SharedPtr node, const runner_opts &run_config, const std::string &bond_id) override
        {
            init_base(node, run_config);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            emit_started(bond_id, "", param_on_started());
        }

        void stop(const std::string &bond_id, const std::string &instance_id = "") override
        {
            if (!node_ || !tf_listener_)
            {
                throw runner_exception("cannot stop runner that was not started");
            }
            emit_stopped(bond_id, instance_id, param_on_stopped());
        }

    protected:
        void execution(capabilities2_events::EventParameters parameters, const std::string &thread_id) override
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
            catch (const tf2::TransformException &error)
            {
                status_message_ = error.what();
                RCLCPP_ERROR(node_->get_logger(), "%s", status_message_.c_str());
                emit_failed(bond_id, instance_id, param_on_failure());
            }
        }

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

    class CurrentJointPoseRunner : public ThreadTriggerRunner
    {
    public:
        CurrentJointPoseRunner() : ThreadTriggerRunner()
        {
        }

        void start(rclcpp::Node::SharedPtr node, const runner_opts &run_config, const std::string &bond_id) override
        {
            init_base(node, run_config);

            std::string topic_name = "/joint_states";
            try
            {
                topic_name = get_topic_name_by_type("sensor_msgs::msg::JointState");
            }
            catch (const runner_exception &)
            {
            }

            joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                topic_name,
                10,
                std::bind(&CurrentJointPoseRunner::joint_state_callback, this, std::placeholders::_1));
            emit_started(bond_id, "", param_on_started());
        }

        void stop(const std::string &bond_id, const std::string &instance_id = "") override
        {
            if (!node_ || !joint_state_subscription_)
            {
                throw runner_exception("cannot stop runner that was not started");
            }
            emit_stopped(bond_id, instance_id, param_on_stopped());
        }

    protected:
        void execution(capabilities2_events::EventParameters parameters, const std::string &thread_id) override
        {
            const std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
            const std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

            const double timeout_sec = std::any_cast<double>(parameters.get_value("timeout_sec", 2.0));
            std::vector<std::string> requested_joint_names;
            try
            {
                requested_joint_names = moveit2_detail::any_to_string_vector(
                    parameters.get_value("joint_names_csv", std::string("")));
            }
            catch (const runner_exception &)
            {
                requested_joint_names.clear();
            }

            const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
            while (std::chrono::steady_clock::now() < deadline)
            {
                std::scoped_lock lock(joint_state_mutex_);
                if (latest_joint_state_)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            sensor_msgs::msg::JointState latest_joint_state;
            {
                std::scoped_lock lock(joint_state_mutex_);
                if (!latest_joint_state_)
                {
                    status_message_ = "No /joint_states message received before timeout.";
                    emit_failed(bond_id, instance_id, param_on_failure());
                    return;
                }
                latest_joint_state = *latest_joint_state_;
            }

            std::vector<std::string> output_joint_names;
            std::vector<double> output_joint_positions;
            if (requested_joint_names.empty())
            {
                output_joint_names = latest_joint_state.name;
                output_joint_positions.assign(latest_joint_state.position.begin(), latest_joint_state.position.end());
            }
            else
            {
                for (const auto &requested_name : requested_joint_names)
                {
                    bool found = false;
                    for (size_t index = 0; index < latest_joint_state.name.size() && index < latest_joint_state.position.size(); ++index)
                    {
                        if (latest_joint_state.name[index] == requested_name)
                        {
                            output_joint_names.push_back(latest_joint_state.name[index]);
                            output_joint_positions.push_back(latest_joint_state.position[index]);
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        status_message_ = "Requested joint not found in latest joint state: " + requested_name;
                        emit_failed(bond_id, instance_id, param_on_failure());
                        return;
                    }
                }
            }

            last_joint_names_csv_ = moveit2_detail::join_csv(output_joint_names);
            last_joint_positions_csv_ = moveit2_detail::join_csv(output_joint_positions);
            last_joint_count_ = static_cast<int>(output_joint_names.size());
            emit_succeeded(bond_id, instance_id, param_on_success());
        }

        capabilities2_events::EventParameters param_on_success() override
        {
            capabilities2_events::EventParameters parameters;
            parameters.set_value("joint_names_csv", last_joint_names_csv_, capabilities2_events::OptionType::STRING);
            parameters.set_value("joint_positions_csv", last_joint_positions_csv_, capabilities2_events::OptionType::STRING);
            parameters.set_value("joint_count", last_joint_count_, capabilities2_events::OptionType::INT);
            return parameters;
        }

        capabilities2_events::EventParameters param_on_failure() override
        {
            capabilities2_events::EventParameters parameters;
            parameters.set_value("message", status_message_, capabilities2_events::OptionType::STRING);
            return parameters;
        }

    private:
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::scoped_lock lock(joint_state_mutex_);
            latest_joint_state_ = msg;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
        std::mutex joint_state_mutex_;
        sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
        std::string last_joint_names_csv_;
        std::string last_joint_positions_csv_;
        std::string status_message_;
        int last_joint_count_{0};
    };
} // namespace capabilities2_runner