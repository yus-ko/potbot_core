#ifndef H_POTBOT_NAV_PARTICLE_SWARM_OPTIMIZATION_
#define H_POTBOT_NAV_PARTICLE_SWARM_OPTIMIZATION_

#include <potbot_lib/particle_swarm_optimization.h>
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
#include <potbot_plugin/ParticleSwarmOptimizationConfig.h>

namespace potbot_nav
{
    namespace path_planner
    {
        class ParticleSwarmOptimization : public potbot_base::PathPlanner
        {
            private:
                ros::Publisher pub_path_, pub_surface_, pub_optimized_, pub_particles_, pub_debug_marker_;

				std::string frame_id_global_ = "map";

                potbot_lib::path_planner::ParticleSwarmOptimization pso_;
                dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig> *dsrv_;

                void reconfigureCB(const potbot_plugin::ParticleSwarmOptimizationConfig& param, uint32_t level);

				void pointvec_to_pcl(const std::vector<potbot_lib::Point>& points_input, pcl::PointCloud<pcl::PointXYZ>& pcl_output);
				void point_to_marker(const potbot_lib::Point& points_input, visualization_msgs::Marker& marker_output);
				void point_to_marker(const Eigen::VectorXd& points_input, visualization_msgs::Marker& marker_output);
				void mat_to_marker_points(const Eigen::MatrixXd& points_input, visualization_msgs::Marker& marker_output);
				void path_to_line_marker(const std::vector<potbot_lib::Pose>& vector_input, visualization_msgs::Marker& marker_output);

                // void publishPath();

            public:
                ParticleSwarmOptimization(){};
                ~ParticleSwarmOptimization(){};

                void initialize(std::string name, tf2_ros::Buffer* tf);
                
                void planPath();
                void setRobot(const nav_msgs::Odometry& pose_msg);
                void setTargetPose(const geometry_msgs::PoseStamped& pose_msg);
                void setObstacle(const geometry_msgs::Point& point_msg);
                void setObstacles(const std::vector<geometry_msgs::Point>& point_msgs);
                void clearObstacles();
                void getPath(std::vector<geometry_msgs::PoseStamped>& path_msg);

        };
    }
}

#endif	// H_POTBOT_NAV_PARTICLE_SWARM_OPTIMIZATION_