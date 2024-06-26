#ifndef H_DIFFDRIVECONTROLLER_ROS_
#define H_DIFFDRIVECONTROLLER_ROS_

#include <potbot_lib/diff_drive_controller.h>
#include <potbot_lib/utility_ros.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/ControllerConfig.h>

namespace potbot_lib{

    namespace controller{

        class DiffDriveControllerROS : public DiffDriveController{
            private:
                ros::Publisher pub_lookahead_;
                std::string frame_id_global_ = "map";
                bool reset_path_index_ = true;
                dynamic_reconfigure::Server<potbot_lib::ControllerConfig> *dsrv_;

                void reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level); 

            public:
                DiffDriveControllerROS(std::string name);
                ~DiffDriveControllerROS(){};

                void setTarget(const geometry_msgs::Pose& pose_msg);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);
                void setTargetPath(const nav_msgs::Path& path_msg);

                void getLookahead(visualization_msgs::Marker& marker_msg);

                void publishLookahead();
        };
    }
}

#endif	// H_DIFFDRIVECONTROLLER_ROS_