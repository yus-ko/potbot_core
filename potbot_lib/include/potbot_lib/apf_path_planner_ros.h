#ifndef H_APF_PATH_PLANNER_ROS_
#define H_APF_PATH_PLANNER_ROS_

#include <potbot_lib/utility_ros.h>
#include <potbot_lib/apf_path_planner.h>
#include <potbot_lib/artificial_potential_field_ros.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/APFPathPlannerConfig.h>

namespace potbot_lib{

    namespace path_planner{

        void getPathMsgFromCsv(nav_msgs::Path& path_msg,const std::string& csv_fullpath);

        class APFPathPlannerROS : public APFPathPlanner{
            private:
                ros::Publisher pub_path_;
                std::string frame_id_global_ = "map";
                dynamic_reconfigure::Server<potbot_lib::APFPathPlannerConfig> *dsrv_;

                void reconfigureCB(const potbot_lib::APFPathPlannerConfig& param, uint32_t level); 

            public:
                // APFPathPlannerROS(ArtificialPotentialField* apf);
                APFPathPlannerROS(std::string name, ArtificialPotentialFieldROS* apf_ros);
                ~APFPathPlannerROS(){};
                
                void getLoopEdges(visualization_msgs::MarkerArray& msg);

                void getPath(std::vector<geometry_msgs::PoseStamped> &msg);
                void getPath(nav_msgs::Path &msg);

                void publishPath();
        };

    }
}

#endif	// H_APF_PATH_PLANNER_ROS_