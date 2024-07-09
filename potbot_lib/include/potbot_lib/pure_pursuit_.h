#ifndef H_POTBOT_PURE_PURSUIT_
#define H_POTBOT_PURE_PURSUIT_

#include <potbot_lib/base_controller.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/ControllerConfig.h>
#include <nav_core/base_local_planner.h>

namespace potbot_lib
{
    namespace controller
    {
        class PurePursuit : public BaseController
        {
            private:
                ros::Publisher pub_lookahead_;
                std::string frame_id_global_ = "map";
                bool reset_path_index_ = true;
                dynamic_reconfigure::Server<potbot_lib::ControllerConfig> *dsrv_;
                boost::shared_ptr<nav_core::BaseLocalPlanner> ddr_;

                void reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level); 

                void getLookahead(visualization_msgs::Marker& marker_msg);
                void publishLookahead();

                void purePursuitController();
                void normalizedPurePursuit();

            public:
                PurePursuit(){};
                ~PurePursuit(){};

                void initialize(std::string name);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);
                void setTargetPath(const nav_msgs::Path& path_msg);

                bool reachedTarget();

        };
    }
}

#endif	// H_POTBOT_PURE_PURSUIT_