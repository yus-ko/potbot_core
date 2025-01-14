#ifndef H_POTBOT_BASE_PATH_PLANNER_
#define H_POTBOT_BASE_PATH_PLANNER_

#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

namespace potbot_base
{
    class PathPlanner
    {
        protected:
            std::string node_name_space_;
            tf2_ros::Buffer* tf_;
            nav_msgs::Odometry robot_pose_;
            geometry_msgs::PoseStamped target_pose_;
            std::vector<geometry_msgs::PoseStamped> path_;
            std::vector<geometry_msgs::Point> obstacles_;
        public:
            virtual void initialize(std::string name, tf2_ros::Buffer* tf){node_name_space_ = name, tf_ = tf;};
            virtual void planPath(){};
            virtual void setRobot(const nav_msgs::Odometry& pose_msg){robot_pose_ = pose_msg;};
            virtual void setTargetPose(const geometry_msgs::PoseStamped& pose_msg){target_pose_ = pose_msg;};
            virtual void setObstacle(const geometry_msgs::Point& point_msg){obstacles_.push_back(point_msg);};
            virtual void setObstacles(const std::vector<geometry_msgs::Point>& point_msgs){obstacles_ = point_msgs;};
            virtual void clearObstacles(){obstacles_.clear();};
            virtual void getObstacles(std::vector<geometry_msgs::Point>& point_msgs){point_msgs = obstacles_;};
            virtual void getPath(std::vector<geometry_msgs::PoseStamped>& path_msg){path_msg = path_;};
    };
}

#endif	// H_POTBOT_BASE_PATH_PLANNER_