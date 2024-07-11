#ifndef H_POTBOT_NAV_TIME_STATE_
#define H_POTBOT_NAV_TIME_STATE_

#include <potbot_lib/time_state.h>
#include <potbot_base/base_controller.h>
#include <potbot_lib/utility_ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_plugin/TimeStateConfig.h>

namespace potbot_nav
{
    namespace controller
    {
        class TimeState : public potbot_base::Controller
        {
            private:
                potbot_lib::controller::TimeState controller_;
                std::string frame_id_global_ = "map";
                dynamic_reconfigure::Server<potbot_plugin::TimeStateConfig> *dsrv_;

                tf2_ros::TransformBroadcaster tf_broadcaster_;

                void reconfigureCB(const potbot_plugin::TimeStateConfig& param, uint32_t level);

            public:
                TimeState(){};
                ~TimeState(){};

                void initialize(std::string name, tf2_ros::Buffer* tf);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);

                bool reachedTarget();

        };
    }
}

#endif	// H_POTBOT_NAV_TIME_STATE_