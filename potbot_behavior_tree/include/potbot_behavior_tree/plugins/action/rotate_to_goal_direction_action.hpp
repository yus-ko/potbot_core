#ifndef POTBOT_NAV_ROTATE_TO_GOAL_DIRECTION_ACTION_HPP_
#define POTBOT_NAV_ROTATE_TO_GOAL_DIRECTION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "potbot_msgs/action/goal_pose.hpp"

namespace potbot_nav
{

    class SpinAction : public nav2_behavior_tree::BtActionNode<potbot_msgs::action::GoalPose>
    {
    public:
        SpinAction(
            const std::string &xml_tag_name,
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        void on_tick() override;

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
                {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to pose"),
                 BT::InputPort<double>("gain_p", 1.0, "P gain"),
                 BT::InputPort<double>("gain_i", 0.1, "I gain"),
                 BT::InputPort<double>("gain_d", 0.01, "D gain")});
        }

    private:
        rclcpp::Logger logger_;
    };

} // namespace potbot_nav

#endif // POTBOT_NAV_ROTATE_TO_GOAL_DIRECTION_ACTION_HPP_
