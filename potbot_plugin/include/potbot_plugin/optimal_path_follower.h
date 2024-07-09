#ifndef H_POTBOT_NAV_OPTIMAL_PATH_FOLLOWER_
#define H_POTBOT_NAV_OPTIMAL_PATH_FOLLOWER_

#include <potbot_base/base_controller.h>
#include <potbot_lib/optimal_path_follower.h>

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/DWAConfig.h>

namespace potbot_nav
{
    namespace controller
    {
        class OptimalPathFollower : public potbot_base::Controller
        {
            private:
                potbot_lib::controller::OptimalPathFollower optimizer_;
                ros::Publisher pub_plans_, pub_best_plan_, pub_split_path_, pub_objective_function_;
                std::string frame_id_global_ = "map";
                dynamic_reconfigure::Server<potbot_lib::DWAConfig> *dsrv_;

                void reconfigureCB(const potbot_lib::DWAConfig& param, uint32_t level);

                void getPlans(visualization_msgs::MarkerArray& msg);
                void getSplitPath(nav_msgs::Path& msg);
                void getBestPath(nav_msgs::Path& msg);
                void getBestCmd(geometry_msgs::Twist& cmd);

                void publishPlans();
                void publishBestPlan();
                void publishSplitPath();

            public:
                OptimalPathFollower(){};
                ~OptimalPathFollower(){};

                void initialize(std::string name);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);

                bool reachedTarget();
        };
    }
}

#endif	// H_POTBOT_NAV_OPTIMAL_PATH_FOLLOWER_