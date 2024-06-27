#ifndef H_POTBOT_BASE_CONTROLLER_
#define H_POTBOT_BASE_CONTROLLER_

#include <potbot_lib/diff_drive_controller.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace potbot_lib
{
    namespace controller
    {
        class BaseController : public DiffDriveController
        {
            public:
            virtual void initialize(std::string name){};
            virtual void calculateCommand(geometry_msgs::Twist& cmd_vel){};
            virtual void setRobot(const nav_msgs::Odometry& pose_msg);
            virtual void setTargetPose(const geometry_msgs::Pose& pose_msg);
            virtual void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);
            virtual void getRobot(nav_msgs::Odometry& pose_msg);
        };
    }
}

#endif	// H_POTBOT_BASE_CONTROLLER_