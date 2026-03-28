// Copyright 2024 potbot
#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "potbot_plugin/hybrid_apf_controller.hpp"

namespace potbot_nav {
namespace controller {

void HybridApfController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_        = parent;
    tf_          = tf;
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros_->getCostmap();

    auto node = node_.lock();
    logger_   = node->get_logger();
    clock_    = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_att",
        rclcpp::ParameterValue(k_att_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_rep",
        rclcpp::ParameterValue(k_rep_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".d_th",
        rclcpp::ParameterValue(d_th_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_v",
        rclcpp::ParameterValue(k_v_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_omega",
        rclcpp::ParameterValue(k_omega_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".v_max",
        rclcpp::ParameterValue(v_max_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".omega_max",
        rclcpp::ParameterValue(omega_max_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".waypoint_tolerance",
        rclcpp::ParameterValue(waypoint_tolerance_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_tolerance",
        rclcpp::ParameterValue(goal_tolerance_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_cost_threshold",
        rclcpp::ParameterValue(obstacle_cost_threshold_));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_obstacle_distance",
        rclcpp::ParameterValue(max_obstacle_distance_));

    k_att_                   = node->get_parameter(plugin_name_ + ".k_att").as_double();
    k_rep_                   = node->get_parameter(plugin_name_ + ".k_rep").as_double();
    d_th_                    = node->get_parameter(plugin_name_ + ".d_th").as_double();
    k_v_                     = node->get_parameter(plugin_name_ + ".k_v").as_double();
    k_omega_                 = node->get_parameter(plugin_name_ + ".k_omega").as_double();
    v_max_                   = node->get_parameter(plugin_name_ + ".v_max").as_double();
    omega_max_               = node->get_parameter(plugin_name_ + ".omega_max").as_double();
    waypoint_tolerance_      = node->get_parameter(plugin_name_ + ".waypoint_tolerance").as_double();
    goal_tolerance_          = node->get_parameter(plugin_name_ + ".goal_tolerance").as_double();
    obstacle_cost_threshold_ = node->get_parameter(plugin_name_ + ".obstacle_cost_threshold").as_double();
    max_obstacle_distance_   = node->get_parameter(plugin_name_ + ".max_obstacle_distance").as_double();

    controller_.setParams(k_att_, k_rep_, d_th_, k_v_, k_omega_,
                          v_max_, omega_max_, waypoint_tolerance_, goal_tolerance_);

    RCLCPP_INFO(logger_, "HybridApfController configured: %s", plugin_name_.c_str());
}

void HybridApfController::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up controller: %s of type HybridApfController",
                plugin_name_.c_str());
}

void HybridApfController::activate()
{
    RCLCPP_INFO(logger_, "Activating controller: %s of type HybridApfController",
                plugin_name_.c_str());
}

void HybridApfController::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating controller: %s of type HybridApfController",
                plugin_name_.c_str());
}

void HybridApfController::setPlan(const nav_msgs::msg::Path& path)
{
    std::vector<potbot_lib::Pose> lib_path = potbot_lib::utility::get_path(
        const_cast<std::vector<geometry_msgs::msg::PoseStamped>&>(path.poses));
    controller_.setGlobalPath(lib_path);
}

void HybridApfController::extractObstaclesFromCostmap(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    std::vector<potbot_lib::Point>& obstacles)
{
    obstacles.clear();

    if (costmap_ == nullptr) {
        return;
    }

    double rx = robot_pose.pose.position.x;
    double ry = robot_pose.pose.position.y;

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();

    for (unsigned int ix = 0; ix < size_x; ix++) {
        for (unsigned int iy = 0; iy < size_y; iy++) {
            unsigned char cost = costmap_->getCost(ix, iy);
            if (static_cast<double>(cost) >= obstacle_cost_threshold_) {
                double wx = 0.0, wy = 0.0;
                costmap_->mapToWorld(ix, iy, wx, wy);
                double dx   = wx - rx;
                double dy   = wy - ry;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= max_obstacle_distance_) {
                    obstacles.push_back(potbot_lib::Point{wx, wy, 0.0});
                }
            }
        }
    }
}

geometry_msgs::msg::TwistStamped HybridApfController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
    (void)velocity;
    (void)goal_checker;

    geometry_msgs::msg::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    std::vector<potbot_lib::Point> obstacles;
    extractObstaclesFromCostmap(robot_pose, obstacles);
    controller_.setObstacles(obstacles);

    potbot_lib::utility::to_agent(robot_pose, controller_);

    controller_.computeCommand();

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = clock_->now();
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.twist.linear.x  = controller_.v;
    cmd_vel.twist.angular.z = controller_.omega;

    return cmd_vel;
}

}  // namespace controller
}  // namespace potbot_nav
