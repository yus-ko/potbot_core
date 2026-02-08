// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "potbot_behavior_tree/plugins/action/rotate_to_goal_direction_action.hpp"

namespace potbot_nav
{

    SpinAction::SpinAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<potbot_msgs::action::GoalPose>(xml_tag_name, action_name, conf),
          logger_(rclcpp::get_logger("RotateToGoalDirection"))
    {

        double p, i, d;
        getInput("gain_p", p);
        getInput("gain_i", i);
        getInput("gain_d", d);

        RCLCPP_INFO(logger_, "Inputed gain: %f, %f, %f", p, i, d);
    }

    void SpinAction::on_tick()
    {
        geometry_msgs::msg::PoseStamped goal_pose;
        getInput("goal", goal_pose);
        goal_.pose = goal_pose;
        RCLCPP_INFO(logger_, "Inputed goal: %f, %f",
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y);
    }

} // namespace potbot_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<potbot_nav::SpinAction>(name, "rotate_to_goal_direction", config);
    };

    factory.registerBuilder<potbot_nav::SpinAction>("RotateToGoalDirection", builder);
}
