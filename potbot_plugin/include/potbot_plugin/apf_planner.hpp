// Copyright 2024 potbot
#ifndef POTBOT_PLUGIN__APF_PLANNER_HPP_
#define POTBOT_PLUGIN__APF_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "potbot_ros/artificial_potential_field.hpp"
#include "potbot_ros/apf_path_planner.hpp"

namespace potbot_nav
{
namespace planner
{

class APF : public nav2_core::GlobalPlanner
{
public:
  APF() = default;
  ~APF() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;

  // 経路計画メソッド ("dijkstra" または "weighted")
  std::string planning_method_;
  // 経路の最大長（メートル）
  double max_path_length_param_;
  // 探索範囲（グリッド数）
  int path_search_range_param_;
  // 局所解脱出時のポテンシャル値の重み
  double weight_potential_;
  // 局所解脱出時の姿勢角度変化の重み
  double weight_pose_;

  // 局所解脱出戦略
  std::string escape_method_;
  // 渦巻き力の回転角度（ラジアン）
  double vortex_angle_;
  // 仮想障害物の生存ステップ数
  int virtual_obstacle_lifetime_;
  // 仮想障害物配置の最大再試行回数
  int max_escape_attempts_;
  // APFフィールド解像度（costmap解像度とは独立）
  double field_resolution_;
  // APFフィールド片側最大サイズ [m]（全体の物理サイズ = 2 * max_field_half_size_）
  double max_field_half_size_;

  std::shared_ptr<potbot_lib::ArtificialPotentialFieldROS> apfros_;
};

}  // namespace planner
}  // namespace potbot_nav

#endif  // POTBOT_PLUGIN__APF_PLANNER_HPP_
