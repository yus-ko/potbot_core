#ifndef POTBOT_NAV_BEHAVIOR_PLUGINS_ROTATE_TO_GOAL_DIRECTION_HPP_
#define POTBOT_NAV_BEHAVIOR_PLUGINS_ROTATE_TO_GOAL_DIRECTION_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "potbot_msgs/action/goal_pose.hpp"

#include "potbot_lib/pid.hpp"
#include "potbot_ros/utility.hpp"

namespace potbot_nav
{
    namespace behavior
    {
        using GoalPoseAction = potbot_msgs::action::GoalPose;

        /**
         * @class nav2_behaviors::Spin
         * @brief An action server behavior for spinning in
         */
        class RotateToGoalDirection : public nav2_behaviors::TimedBehavior<GoalPoseAction>
        {
        public:
            /**
             * @brief A constructor for nav2_behaviors::Spin
             */
            RotateToGoalDirection();
            ~RotateToGoalDirection();

            /**
             * @brief Initialization to run behavior
             * @param command Goal to execute
             * @return Status of behavior
             */
            nav2_behaviors::Status onRun(const std::shared_ptr<const GoalPoseAction::Goal> command) override;

            /**
             * @brief Configuration of behavior action
             */
            void onConfigure() override;

            /**
             * @brief Loop function to run behavior
             * @return Status of behavior
             */
            nav2_behaviors::Status onCycleUpdate() override;

        protected:
            GoalPoseAction::Feedback::SharedPtr feedback_;

            rclcpp::Duration command_time_allowance_{0, 0};
            rclcpp::Time end_time_;

            double gain_p_;
            double gain_i_;
            double gain_d_;
            double tolerance_angle_;

            geometry_msgs::msg::PoseStamped goal_pose_;
            potbot_lib::controller::PID controller_;
        };
    }

} // namespace potbot_nav

#endif // POTBOT_NAV_BEHAVIORS_PLUGINS_ROTATE_TO_GOAL_DIRECTION_HPP_
