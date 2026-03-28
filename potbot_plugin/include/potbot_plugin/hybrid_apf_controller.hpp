// Copyright 2024 potbot
#ifndef POTBOT_PLUGIN__HYBRID_APF_CONTROLLER_HPP_
#define POTBOT_PLUGIN__HYBRID_APF_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "potbot_lib/apf_waypoint_controller.hpp"
#include "potbot_ros/utility.hpp"

namespace potbot_nav {
namespace controller {

class HybridApfController : public nav2_core::Controller {
public:
    HybridApfController() = default;
    ~HybridApfController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override {}

    void setPlan(const nav_msgs::msg::Path& path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

private:
    void extractObstaclesFromCostmap(
        const geometry_msgs::msg::PoseStamped& robot_pose,
        std::vector<potbot_lib::Point>& obstacles);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
    rclcpp::Logger logger_{rclcpp::get_logger("HybridApfController")};
    rclcpp::Clock::SharedPtr clock_;

    potbot_lib::controller::ApfWaypointController controller_;

    double k_att_                   = 1.0;
    double k_rep_                   = 2.0;
    double d_th_                    = 0.5;
    double k_v_                     = 0.5;
    double k_omega_                 = 2.0;
    double v_max_                   = 0.22;
    double omega_max_               = 1.5;
    double waypoint_tolerance_      = 0.2;
    double goal_tolerance_          = 0.05;
    double obstacle_cost_threshold_ = 200.0;
    double max_obstacle_distance_   = 2.0;
};

}  // namespace controller
}  // namespace potbot_nav

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::HybridApfController, nav2_core::Controller)

#endif  // POTBOT_PLUGIN__HYBRID_APF_CONTROLLER_HPP_
