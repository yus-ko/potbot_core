#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "potbot_behavior_tree/plugins/rotate_to_goal_direction.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace potbot_nav
{
    namespace behavior
    {

        RotateToGoalDirection::RotateToGoalDirection()
            : nav2_behaviors::TimedBehavior<GoalPoseAction>(),
              feedback_(std::make_shared<GoalPoseAction::Feedback>()),
              gain_p_(1.0),
              gain_i_(1.0),
              gain_d_(1.0),
              tolerance_angle_(0.01)
        {
        }

        RotateToGoalDirection::~RotateToGoalDirection() = default;

        void RotateToGoalDirection::onConfigure()
        {
            auto node = node_.lock();
            if (!node)
            {
                throw std::runtime_error{"Failed to lock node"};
            }

            nav2_util::declare_parameter_if_not_declared(
                node, "gain_p", rclcpp::ParameterValue(1.0));
            node->get_parameter("gain_p", gain_p_);

            nav2_util::declare_parameter_if_not_declared(
                node, "gain_i", rclcpp::ParameterValue(0.1));
            node->get_parameter("gain_i", gain_i_);

            nav2_util::declare_parameter_if_not_declared(
                node, "gain_d", rclcpp::ParameterValue(0.01));
            node->get_parameter("gain_d", gain_d_);

            nav2_util::declare_parameter_if_not_declared(
                node, "tolerance_angle", rclcpp::ParameterValue(0.01));
            node->get_parameter("tolerance_angle", tolerance_angle_);
        }

        nav2_behaviors::Status RotateToGoalDirection::onRun(const std::shared_ptr<const GoalPoseAction::Goal> command)
        {
            geometry_msgs::msg::PoseStamped current_pose;
            if (!nav2_util::getCurrentPose(
                    current_pose, *tf_, global_frame_, robot_base_frame_,
                    transform_tolerance_))
            {
                RCLCPP_ERROR(logger_, "Current robot pose is not available.");
                return nav2_behaviors::Status::FAILED;
            }

            goal_pose_ = command->pose;

            // cmd_yaw_ = command->target_yaw;
            RCLCPP_INFO(
                logger_, "Turning to (%0.2f, %0.2f) for RotateToGoalDirection behavior.",
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);

            command_time_allowance_ = command->time_allowance;
            end_time_ = this->clock_->now() + command_time_allowance_;

            controller_.setTargetPoint(
                potbot_lib::utility::get_pose(goal_pose_.pose));
            controller_.initPID();
            controller_.setGain(gain_p_, gain_i_, gain_d_);
            controller_.setLimit(0.3, 1.5);
            controller_.setMargin(tolerance_angle_, 0.3);

            return nav2_behaviors::Status::SUCCEEDED;
        }

        nav2_behaviors::Status RotateToGoalDirection::onCycleUpdate()
        {
            rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
            if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
            {
                stopRobot();
                RCLCPP_WARN(
                    logger_,
                    "Exceeded time allowance before reaching the Spin goal - Exiting Spin");
                return nav2_behaviors::Status::FAILED;
            }

            geometry_msgs::msg::PoseStamped current_pose;
            if (!nav2_util::getCurrentPose(
                    current_pose, *tf_, global_frame_, robot_base_frame_,
                    transform_tolerance_))
            {
                RCLCPP_ERROR(logger_, "Current robot pose is not available.");
                return nav2_behaviors::Status::FAILED;
            }

            potbot_lib::utility::to_agent(current_pose, controller_);
            controller_.calculateCommand();

            action_server_->publish_feedback(feedback_);

            double angle_current = controller_.yaw;
            double angle_target = controller_.getAngle(
                potbot_lib::utility::get_point(goal_pose_.pose.position));
            double angle_error = angle_target - angle_current;
            double tolerance = 0.01;
            RCLCPP_DEBUG(logger_, "pose: %f, target: %f, error/tolerance: %f / %f, command: %f",
                         angle_current, angle_target, angle_error, tolerance, controller_.omega);
            if (abs(angle_error) < tolerance)
            {
                stopRobot();
                return nav2_behaviors::Status::SUCCEEDED;
            }

            auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

            geometry_msgs::msg::Pose2D pose2d;
            pose2d.x = current_pose.pose.position.x;
            pose2d.y = current_pose.pose.position.y;
            pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

            // cmd_vel->linear.x = controller_.v;
            cmd_vel->angular.z = controller_.omega;
            vel_pub_->publish(std::move(cmd_vel));

            return nav2_behaviors::Status::RUNNING;
        }
    } // namespace behavior

} // namespace potbot_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potbot_nav::behavior::RotateToGoalDirection, nav2_core::Behavior)
