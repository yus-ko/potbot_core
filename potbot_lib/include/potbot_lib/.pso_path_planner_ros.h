#ifndef H_PSO_PATH_PLANNER_ROS_
#define H_PSO_PATH_PLANNER_ROS_

#include <potbot_lib/utility_ros.h>
#include <potbot_lib/particle_swarm_optimization.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/PSOPathPlannerConfig.h>

namespace potbot_lib
{
    namespace path_planner
	{
        class PSOPathPlannerROS : public ParticleSwarmOptimization
		{
			protected:
				ros::Publisher pub_path_, pub_surface_, pub_optimized_, pub_particles_, pub_debug_marker_;

				std::string frame_id_global_ = "map";

				dynamic_reconfigure::Server<potbot_lib::PSOPathPlannerConfig> *dsrv_;

				void reconfigureCB(const potbot_lib::PSOPathPlannerConfig& param, uint32_t level);

				void pointvec_to_pcl(const std::vector<Point>& points_input, pcl::PointCloud<pcl::PointXYZ>& pcl_output);
				void point_to_marker(const Point& points_input, visualization_msgs::Marker& marker_output);
				void point_to_marker(const Eigen::VectorXd& points_input, visualization_msgs::Marker& marker_output);
				void mat_to_marker_points(const Eigen::MatrixXd& points_input, visualization_msgs::Marker& marker_output);
				void path_to_line_marker(const std::vector<Pose>& vector_input, visualization_msgs::Marker& marker_output);

			public:
				PSOPathPlannerROS(std::string name);
                ~PSOPathPlannerROS(){};

				void setGoal(const geometry_msgs::PoseStamped& goal);

				void setRobot(const geometry_msgs::Pose& robot);
				void setRobot(const geometry_msgs::PoseStamped& robot);
				void setRobot(const nav_msgs::Odometry& robot);
				
				void setObstacle(const visualization_msgs::Marker& obs);
				void setObstacle(const std::vector<visualization_msgs::Marker>& obs);
				void setObstacle(const geometry_msgs::Point& obs);
				void setObstacle(const std::vector<geometry_msgs::Point>& obs);
				void setObstacle(const geometry_msgs::PointStamped& obs);
				void setObstacle(const std::vector<geometry_msgs::PointStamped>& obs);
				void setObstacle(const geometry_msgs::Pose& obs);
				void setObstacle(const std::vector<geometry_msgs::Pose>& obs);
				void setObstacle(const geometry_msgs::PoseStamped& obs);
				void setObstacle(const std::vector<geometry_msgs::PoseStamped>& obs);

				void createPath();

				void getPath(std::vector<geometry_msgs::PoseStamped> &msg);
                void getPath(nav_msgs::Path &msg);

				void publishPath();

		};
	}
}

#endif	// H_PSO_PATH_PLANNER_ROS_
