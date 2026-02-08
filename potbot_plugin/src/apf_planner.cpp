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
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
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
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
      nav_msgs::msg::Path global_path;

      apfros_->initPotentialField(costmap_ros_);
      apfros_->createPotentialField();
      apfros_->publishPotentialField();

      // Checking if the goal and start state is in the global frame
      if (start.header.frame_id != global_frame_)
      {
        RCLCPP_ERROR(
            node_->get_logger(), "Planner will only except start position from %s frame",
            global_frame_.c_str());
        return global_path;
      }

      if (goal.header.frame_id != global_frame_)
      {
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

      for (int i = 0; i < total_number_of_loop; ++i)
      {
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

  } // namespace planner
} // namespace potbot_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potbot_nav::planner::APF, nav2_core::GlobalPlanner)
