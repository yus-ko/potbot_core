#include <potbot_plugin/particle_swarm_optimization.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::path_planner::ParticleSwarmOptimization, potbot_base::PathPlanner)
namespace potbot_nav
{
    namespace path_planner
    {
        void ParticleSwarmOptimization::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            tf_ = tf;
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
			pub_path_ = private_nh.advertise<nav_msgs::Path>("path", 1);
			pub_surface_ = private_nh.advertise<sensor_msgs::PointCloud2>("surface", 1);
			// pub_optimized_ = private_nh.advertise<visualization_msgs::Marker>("optimized", 1);
			// pub_particles_ = private_nh.advertise<visualization_msgs::Marker>("particles", 1);
			pub_debug_marker_ = private_nh.advertise<visualization_msgs::MarkerArray>("debug_markers", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig>::CallbackType cb = boost::bind(&ParticleSwarmOptimization::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void ParticleSwarmOptimization::reconfigureCB(const potbot_plugin::ParticleSwarmOptimizationConfig& param, uint32_t level)
        {
            pso_.setCalculationParam(param.particle_num, param.max_iteration);
			pso_.setWeight(param.weight_velocity, param.weight_pbest, param.weight_gbest);
			pso_.setThresholdDistance(param.threshold_distance_to_obstacle);
        }

        void ParticleSwarmOptimization::setRobot(const nav_msgs::Odometry& pose_msg)
        {
            potbot_lib::Pose p;
            p.position.x = pose_msg.pose.pose.position.x;
            p.position.y = pose_msg.pose.pose.position.y;
            p.rotation.z = tf2::getYaw(pose_msg.pose.pose.orientation);
            pso_.setRobot(p);
        }

        void ParticleSwarmOptimization::setTargetPose(const geometry_msgs::PoseStamped& pose_msg)
        {
            // target_pose_ = pose_msg;
            potbot_lib::Pose p;
            p.position.x = pose_msg.pose.position.x;
            p.position.y = pose_msg.pose.position.y;
            p.rotation.z = tf2::getYaw(pose_msg.pose.orientation);
            pso_.setGoal(p);
        }

        void ParticleSwarmOptimization::setObstacle(const geometry_msgs::Point& point_msg)
        {
            pso_.setObstacle(potbot_lib::Point(point_msg.x, point_msg.y));
        }

        void ParticleSwarmOptimization::setObstacles(const std::vector<geometry_msgs::Point>& point_msgs)
        {
            for (const auto& o : point_msgs)
            {
                setObstacle(o);
            }
        }

        void ParticleSwarmOptimization::clearObstacles()
        {
            pso_.clearObstacles();
        }

        void ParticleSwarmOptimization::getPath(std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            path_msg.clear();
            std::vector<potbot_lib::Pose> path;
            pso_.getPath(path);
            for (const auto p : path)
            {
                geometry_msgs::PoseStamped msg;
                msg.pose = potbot_lib::utility::get_pose(p);
                path_msg.push_back(msg);
            }
        }

        void ParticleSwarmOptimization::planPath()
        {
            pso_.initialize();

            std::vector<potbot_lib::Point> points;
			pso_.createSurface(points,ros::Time::now().toSec());
			pcl::PointCloud<pcl::PointXYZ> pcl;
			pointvec_to_pcl(points, pcl);

			sensor_msgs::PointCloud2 pcl_msg;
			pcl::toROSMsg(pcl, pcl_msg);
			pcl_msg.header.frame_id = frame_id_global_;
			pub_surface_.publish(pcl_msg);

			pso_.createPath();
            nav_msgs::Path path_msg;
            getPath(path_msg.poses);
            path_msg.header.frame_id = frame_id_global_;
            path_msg.header.stamp = ros::Time::now();
            pub_path_.publish(path_msg);

			visualization_msgs::MarkerArray vmamsg_debug_markers;
			visualization_msgs::Marker vmmsg_gbest_trajectory;
			std::vector<potbot_lib::Pose> path;
            std::vector<Eigen::VectorXd> best_history;
            pso_.getBestHistory(best_history);
			potbot_lib::utility::vec_to_path(best_history, path);
			path_to_line_marker(path, vmmsg_gbest_trajectory);
			vmmsg_gbest_trajectory.ns = "trajectory/gbest";
			vmamsg_debug_markers.markers.push_back(vmmsg_gbest_trajectory);

            std::vector<std::vector<Eigen::VectorXd>> particles_history;
            pso_.getParticlesHistory(particles_history);
			for (size_t i = 0; i < particles_history.size(); i++)
			{
				visualization_msgs::Marker vmmsg_particles_trajectory;
				std::vector<potbot_lib::Pose> path;
				potbot_lib::utility::vec_to_path(particles_history[i], path);
				path_to_line_marker(path, vmmsg_particles_trajectory);
				vmmsg_particles_trajectory.id = i;
				vmmsg_particles_trajectory.color = potbot_lib::color::get_msg("lb");
				vmmsg_particles_trajectory.ns = "trajectory/particles";
				vmamsg_debug_markers.markers.push_back(vmmsg_particles_trajectory);
				
			}
			pub_debug_marker_.publish(vmamsg_debug_markers);
        }

        void ParticleSwarmOptimization::pointvec_to_pcl(const std::vector<potbot_lib::Point>& points_input, pcl::PointCloud<pcl::PointXYZ>& pcl_output)
		{
			pcl::PointCloud<pcl::PointXYZ> pcl_init;
			pcl_output = pcl_init;
			pcl_output.width = points_input.size();
			pcl_output.height = 1; // 単一の点群
			pcl_output.points.resize(pcl_output.width * pcl_output.height);

			// 点のベクトルをPointCloudにコピー
			for (size_t i = 0; i < points_input.size(); ++i) {
				pcl_output.points[i].x = points_input[i].x;
				pcl_output.points[i].y = points_input[i].y;
				pcl_output.points[i].z = points_input[i].z;
			}
		}

        void ParticleSwarmOptimization::point_to_marker(const potbot_lib::Point& points_input, visualization_msgs::Marker& marker_output)
		{
            double g_resolution = 0.1;
			marker_output.type = visualization_msgs::Marker::SPHERE;
			marker_output.color.r = 1;
			marker_output.color.g = 1;
			marker_output.color.b = 1;
			marker_output.color.a = 1;
			marker_output.pose.position.x = points_input.x;
			marker_output.pose.position.y = points_input.y;
			marker_output.pose.position.z = points_input.z;
			marker_output.pose.orientation.x = 0;
			marker_output.pose.orientation.y = 0;
			marker_output.pose.orientation.z = 0;
			marker_output.pose.orientation.w = 1;
			marker_output.scale.x = 1.5*g_resolution;
			marker_output.scale.y = 1.5*g_resolution;
			marker_output.scale.z = 1.5*g_resolution;
		}

		void ParticleSwarmOptimization::point_to_marker(const Eigen::VectorXd& points_input, visualization_msgs::Marker& marker_output)
		{
			point_to_marker(potbot_lib::Point(points_input(0), points_input(1), points_input(2)), marker_output);
		}

		void ParticleSwarmOptimization::mat_to_marker_points(const Eigen::MatrixXd& points_input, visualization_msgs::Marker& marker_output)
		{
            double g_resolution = 0.1;
			marker_output.type = visualization_msgs::Marker::SPHERE_LIST;
			marker_output.color.r = 0.1;
			marker_output.color.g = 0.1;
			marker_output.color.b = 0.1;
			marker_output.color.a = 0.7;
			marker_output.pose.position.x = 0;
			marker_output.pose.position.y = 0;
			marker_output.pose.position.z = 0;
			marker_output.pose.orientation.x = 0;
			marker_output.pose.orientation.y = 0;
			marker_output.pose.orientation.z = 0;
			marker_output.pose.orientation.w = 1;
			marker_output.scale.x = g_resolution;
			marker_output.scale.y = g_resolution;
			marker_output.scale.z = g_resolution;
			marker_output.points.clear();
			marker_output.colors.clear();
			for (size_t i = 0; i < points_input.cols(); i++)
			{
				geometry_msgs::Point p;
				p.x = points_input(0,i);
				p.y = points_input(1,i);
				// p.z = objective_function(points_input(0,i), points_input(1,i));
				marker_output.points.push_back(p);
				marker_output.colors.push_back(marker_output.color);
			}
			
		}

		void ParticleSwarmOptimization::path_to_line_marker(const std::vector<potbot_lib::Pose>& vector_input, visualization_msgs::Marker& marker_output)
		{
			marker_output.header.frame_id = frame_id_global_;
			marker_output.header.stamp = ros::Time::now();
			marker_output.type = visualization_msgs::Marker::LINE_STRIP;
			marker_output.color = potbot_lib::color::get_msg("k");
			marker_output.pose = potbot_lib::utility::get_pose();
			for (const auto& vec:vector_input)
			{
				marker_output.points.push_back(
					potbot_lib::utility::get_point(vec.position)
					);
			}
			marker_output.scale.x = 0.03;
		}
    }
}