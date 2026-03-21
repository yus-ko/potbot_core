// Copyright 2024 potbot
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "potbot_plugin/apf_planner.hpp"

namespace potbot_nav
{
namespace planner
{
void APF::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  apfros_ = std::make_shared<potbot_lib::ArtificialPotentialFieldROS>(node_);

  // apf_ = std::make_shared<potbot_lib::ArtificialPotentialField>(
  //     100,
  //     100,
  //     0.05
  //     // double weight_attraction_field = (0.1),
  //     // double weight_repulsion_field = (0.1),
  //     // double distance_threshold_repulsion_field = (0.3),
  //     // double field_origin_x = (0.0),
  //     // double field_origin_y = (0.0)
  //   );

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void APF::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void APF::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void APF::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path APF::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // apfros_->initPotentialField(costmap_ros_);
  // apfros_->clearObstacles();

  // const auto robot = apfros_->getApf()->getRobot();

  geometry_msgs::msg::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  const potbot_lib::Point robot = potbot_lib::utility::get_point(robot_pose.pose.position);

  // ロボット-ゴール間の距離に基づいてフィールドサイズを動的に計算し、
  // ゴールが必ずフィールド内に含まれるようにする。
  // ゴールがフィールド外の場合、IS_AROUND_GOALが設定されず経路がループする原因となる。
  const double resolution = 0.05;
  const int max_half_cells = 100;  // 最大 10m x 10m
  double dist_x = std::abs(goal.pose.position.x - robot.x);
  double dist_y = std::abs(goal.pose.position.y - robot.y);
  double max_dist = std::max({dist_x, dist_y, 1.25});
  int half_cells = std::min(static_cast<int>(max_dist / resolution) + 5, max_half_cells);
  int total_cells = 2 * half_cells;

  apfros_->getApf()->initPotentialField(total_cells, total_cells, resolution, robot.x, robot.y);
  apfros_->setRobot(robot_pose);
  apfros_->setGoal(goal);

  unsigned int rmx, rmy;
  costmap_->worldToMap(robot.x, robot.y, rmx, rmy);

  for (int mx = rmx - half_cells; mx < rmx + half_cells; mx++) {
    for (int my = rmy - half_cells; my < rmy + half_cells; my++) {
      if (mx < 0 || my < 0) {
        continue;
      }
      const auto c = costmap_->getCost(mx, my);
      if (c == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double x, y;
        costmap_->mapToWorld(mx, my, x, y);
        apfros_->setObstacle(potbot_lib::utility::get_point(x, y));
      }
    }
  }

  apfros_->createPotentialField();
  apfros_->publishPotentialField();

  std::shared_ptr<potbot_lib::path_planner::APFPathPlannerROS> planner =
    std::make_shared<potbot_lib::path_planner::APFPathPlannerROS>(apfros_);
  planner->createPath();
  // planner->publishPath();
  // planner->publishRawPath();

  global_path.poses.clear();
  planner->getPath(global_path);
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  return global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace planner
}  // namespace potbot_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potbot_nav::planner::APF, nav2_core::GlobalPlanner)
