#ifndef H_POTBOT_NAV_ARTIFICIAL_POTENTIAL_FIELD_
#define H_POTBOT_NAV_ARTIFICIAL_POTENTIAL_FIELD_

#include <potbot_lib/artificial_potential_field_ros.h>
#include <potbot_lib/apf_path_planner_ros.h>
#include <potbot_base/base_path_planner.h>
#include <potbot_lib/utility_ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <potbot_plugin/ParticleSwarmOptimizationConfig.h>

namespace potbot_nav
{
    namespace path_planner
    {
        class APFPP : public potbot_base::PathPlanner
        {
            private:
                ros::Publisher pub_path_, pub_surface_, pub_optimized_, pub_particles_, pub_debug_marker_;

				std::string frame_id_global_ = "map";

                potbot_lib::ArtificialPotentialFieldROS* apf_ = nullptr;
                potbot_lib::path_planner::APFPathPlannerROS* apfpp_ = nullptr;

                // dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig> *dsrv_;
                // void reconfigureCB(const potbot_plugin::ParticleSwarmOptimizationConfig& param, uint32_t level);

                // void publishPath();

            public:
                APFPP(){};
                ~APFPP(){};

                void initialize(std::string name, tf2_ros::Buffer* tf);
                
                void planPath();
                // void setRobot(const nav_msgs::Odometry& pose_msg);
                // void setTargetPose(const geometry_msgs::PoseStamped& pose_msg);
                // void setObstacle(const geometry_msgs::Point& point_msg);
                // void setObstacles(const std::vector<geometry_msgs::Point>& point_msgs);
                // void clearObstacles();
                void getPath(std::vector<geometry_msgs::PoseStamped>& path_msg);

        };
    }
}

#endif	// H_POTBOT_NAV_ARTIFICIAL_POTENTIAL_FIELD_