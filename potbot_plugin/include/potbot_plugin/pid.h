#ifndef H_POTBOT_NAV_POSITION_PID_
#define H_POTBOT_NAV_POSITION_PID_

#include <potbot_lib/pid.h>
#include <potbot_base/base_controller.h>
#include <potbot_lib/utility_ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_plugin/PIDConfig.h>

namespace potbot_nav
{
    namespace controller
    {
        class PID : public potbot_base::Controller
        {
            private:
                potbot_lib::controller::PID pid_;
                dynamic_reconfigure::Server<potbot_plugin::PIDConfig> *dsrv_;

                void reconfigureCB(const potbot_plugin::PIDConfig& param, uint32_t level); 

            public:
                PID(){};
                ~PID(){};

                void initialize(std::string name, tf2_ros::Buffer* tf);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPose(const geometry_msgs::PoseStamped& pose_msg);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);

                bool reachedTarget();

        };
    }
}

#endif	// H_POTBOT_NAV_POSITION_PID_