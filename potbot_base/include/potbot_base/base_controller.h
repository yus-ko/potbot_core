#ifndef H_POTBOT_BASE_CONTROLLER_
#define H_POTBOT_BASE_CONTROLLER_

#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

namespace potbot_base
{
    class Controller
    {
        protected:
            std::string node_name_space_;
            tf2_ros::Buffer* tf_;
            nav_msgs::Odometry robot_pose_;
            geometry_msgs::PoseStamped target_pose_;
            std::vector<geometry_msgs::PoseStamped> target_path_;
        public:
            virtual void initialize(std::string name, tf2_ros::Buffer* tf){node_name_space_ = name, tf_ = tf;};
            virtual void calculateCommand(geometry_msgs::Twist& cmd_vel){};
            virtual void setRobot(const nav_msgs::Odometry& pose_msg){robot_pose_ = pose_msg;};
            virtual void setTargetPose(const geometry_msgs::PoseStamped& pose_msg){target_pose_ = pose_msg;};
            virtual void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg){target_path_ = path_msg;};
            virtual void getRobot(nav_msgs::Odometry& pose_msg){pose_msg = robot_pose_;};
            virtual bool reachedTarget(){return true;};
    };
}

#endif	// H_POTBOT_BASE_CONTROLLER_