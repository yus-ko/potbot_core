#include <potbot_lib/pso_path_planner_ros.h>

namespace potbot_lib
{
    namespace path_planner
	{
		PSOPathPlannerROS::PSOPathPlannerROS(std::string name)
		{
			ros::NodeHandle private_nh("~/" + name);
			private_nh.getParam("frame_id_global",           frame_id_global_);
			pub_path_ = private_nh.advertise<nav_msgs::Path>("path", 1);
			pub_surface_ = private_nh.advertise<sensor_msgs::PointCloud2>("surface", 1);
			pub_optimized_ = private_nh.advertise<visualization_msgs::Marker>("optimized", 1);
			pub_particles_ = private_nh.advertise<visualization_msgs::Marker>("particles", 1);
			pub_debug_marker_ = private_nh.advertise<visualization_msgs::MarkerArray>("debug_markers", 1);

			dsrv_ = new dynamic_reconfigure::Server<potbot_lib::PSOPathPlannerConfig>(private_nh);
			dynamic_reconfigure::Server<potbot_lib::PSOPathPlannerConfig>::CallbackType cb = boost::bind(&PSOPathPlannerROS::reconfigureCB, this, _1, _2);
			dsrv_->setCallback(cb);
		}

		void PSOPathPlannerROS::reconfigureCB(const potbot_lib::PSOPathPlannerConfig& param, uint32_t level)
        {
			if (particle_num_ != param.particle_num)
			{
				initialize();
			}
            particle_num_ = param.particle_num;
            max_iteration_ = param.max_iteration;
            weight_velocity_ = param.weight_velocity;
            weight_pbest_ = param.weight_pbest;
			weight_gbest_ = param.weight_gbest;
			threshold_distance_to_obstacle_ = param.threshold_distance_to_obstacle;
        }

		void PSOPathPlannerROS::createPath()
		{
			std::vector<Point> points;
			create_surface(points,ros::Time::now().toSec());
			pcl::PointCloud<pcl::PointXYZ> pcl;
			pointvec_to_pcl(points, pcl);

			sensor_msgs::PointCloud2 pcl_msg;
			pcl::toROSMsg(pcl, pcl_msg);
			pcl_msg.header.frame_id = frame_id_global_;
			pub_surface_.publish(pcl_msg);

			ParticleSwarmOptimization::createPath();

			visualization_msgs::MarkerArray vmamsg_debug_markers;
			visualization_msgs::Marker vmmsg_gbest_trajectory;
			std::vector<Pose> path;
			potbot_lib::utility::vec_to_path(gbest_history_, path);
			path_to_line_marker(path, vmmsg_gbest_trajectory);
			vmmsg_gbest_trajectory.ns = "trajectory/gbest";
			vmamsg_debug_markers.markers.push_back(vmmsg_gbest_trajectory);
			for (size_t i = 0; i < particles_history_.size(); i++)
			{
				visualization_msgs::Marker vmmsg_particles_trajectory;
				std::vector<Pose> path;
				potbot_lib::utility::vec_to_path(particles_history_[i], path);
				path_to_line_marker(path, vmmsg_particles_trajectory);
				vmmsg_particles_trajectory.id = i;
				vmmsg_particles_trajectory.color = potbot_lib::color::get_msg("lb");
				vmmsg_particles_trajectory.ns = "trajectory/particles";
				vmamsg_debug_markers.markers.push_back(vmmsg_particles_trajectory);
				
			}
			pub_debug_marker_.publish(vmamsg_debug_markers);
		}

		void PSOPathPlannerROS::pointvec_to_pcl(const std::vector<Point>& points_input, pcl::PointCloud<pcl::PointXYZ>& pcl_output)
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

		void PSOPathPlannerROS::point_to_marker(const Point& points_input, visualization_msgs::Marker& marker_output)
		{
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

		void PSOPathPlannerROS::point_to_marker(const Eigen::VectorXd& points_input, visualization_msgs::Marker& marker_output)
		{
			point_to_marker(Point(points_input(0), points_input(1), points_input(2)), marker_output);
		}

		void PSOPathPlannerROS::mat_to_marker_points(const Eigen::MatrixXd& points_input, visualization_msgs::Marker& marker_output)
		{
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
				p.z = objective_function(points_input(0,i), points_input(1,i));
				marker_output.points.push_back(p);
				marker_output.colors.push_back(marker_output.color);
			}
			
		}

		void PSOPathPlannerROS::path_to_line_marker(const std::vector<Pose>& vector_input, visualization_msgs::Marker& marker_output)
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

		void PSOPathPlannerROS::setGoal(const geometry_msgs::PoseStamped& goal)
		{
			ParticleSwarmOptimization::setGoal(Pose(goal.pose.position.x, goal.pose.position.y));
		}

		void PSOPathPlannerROS::setRobot(const geometry_msgs::Pose& robot)
		{
			ParticleSwarmOptimization::setRobot(Pose(robot.position.x, robot.position.y));
		}

		void PSOPathPlannerROS::setRobot(const geometry_msgs::PoseStamped& robot)
		{
			setRobot(robot.pose);
		}

		void PSOPathPlannerROS::setRobot(const nav_msgs::Odometry& robot)
		{
			setRobot(robot.pose.pose);
		}

		void PSOPathPlannerROS::setObstacle(const visualization_msgs::Marker& obs)
		{
			// return;
			double origin_x = obs.pose.position.x;
			double origin_y = obs.pose.position.y;
			double origin_th = tf2::getYaw(obs.pose.orientation);
			double res = 0.05;

			Eigen::MatrixXd vertexes;
			if (obs.type == visualization_msgs::Marker::CUBE)
			{
				double width = obs.scale.x;
				double height = obs.scale.y;

				Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
				Eigen::MatrixXd translation(4,2);
				translation <<  origin_x, origin_y,
								origin_x, origin_y,
								origin_x, origin_y,
								origin_x, origin_y;
				Eigen::MatrixXd origin_vertexes(4,2);
				origin_vertexes <<  -width/2,   -height/2,
									-width/2,   height/2,
									width/2,    height/2,
									width/2,    -height/2;

				vertexes = rotation*origin_vertexes.transpose() + translation.transpose();
				
			}
			else if (obs.type == visualization_msgs::Marker::SPHERE)
			{
				double width = obs.scale.x;
				double height = obs.scale.y;

				Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
				Eigen::Vector2d translation;
				translation <<  origin_x, origin_y;
				Eigen::MatrixXd origin_vertexes(4,2);
				
				size_t vertex_num = 2*M_PI/res;
				vertexes.resize(2,vertex_num);
				for (size_t i = 0; i < vertex_num; i++)
				{
					double t = 2 * M_PI * i / vertex_num;
					double x = width/2 * cos(t);
					double y = height/2 * sin(t);
					Eigen::Vector2d p;
					p<< x,y;
					vertexes.col(i) = rotation*p + translation;
				}
			}

			for (size_t i = 0; i < vertexes.cols(); i++)
			{
				// std::cout<<vertexes.row(i)<<std::endl;
				ParticleSwarmOptimization::setObstacle(Point(vertexes(0,i), vertexes(1,i)));
			}
		}

		void PSOPathPlannerROS::setObstacle(const std::vector<visualization_msgs::Marker>& obs)
		{
			for (const auto& o:obs)
			{
				setObstacle(o);
			}
		}
		
		void PSOPathPlannerROS::setObstacle(const geometry_msgs::Point& obs)
		{
			ParticleSwarmOptimization::setObstacle(Point(obs.x, obs.y));
		}

		void PSOPathPlannerROS::setObstacle(const std::vector<geometry_msgs::Point>& obs)
		{
			for (const auto& o:obs)
			{
				setObstacle(o);
			}
		}

		void PSOPathPlannerROS::setObstacle(const geometry_msgs::PointStamped& obs)
		{
			setObstacle(obs.point);
		}

		void PSOPathPlannerROS::setObstacle(const std::vector<geometry_msgs::PointStamped>& obs)
		{
			for (const auto& o:obs)
			{
				setObstacle(o);
			}
		}

		void PSOPathPlannerROS::setObstacle(const geometry_msgs::Pose& obs)
		{
			setObstacle(obs.position);
		}

		void PSOPathPlannerROS::setObstacle(const std::vector<geometry_msgs::Pose>& obs)
		{
			for (const auto& o:obs)
			{
				setObstacle(o);
			}
		}

		void PSOPathPlannerROS::setObstacle(const geometry_msgs::PoseStamped& obs)
		{
			setObstacle(obs.pose);
		}

		void PSOPathPlannerROS::setObstacle(const std::vector<geometry_msgs::PoseStamped>& obs)
		{
			for (const auto& o:obs)
			{
				setObstacle(o);
			}
		}

		void PSOPathPlannerROS::getPath(std::vector<geometry_msgs::PoseStamped> &msg)
        {
            msg.clear();
			std::vector<Pose> path;
			ParticleSwarmOptimization::getPath(path);
            for (const auto& point : path)
            {
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.pose = potbot_lib::utility::get_pose(point);
                msg.push_back(pose_msg);
            }
        }

        void PSOPathPlannerROS::getPath(nav_msgs::Path &msg)
        {
            getPath(msg.poses);
        }

		void PSOPathPlannerROS::publishPath()
        {
            nav_msgs::Path path_msg;
            getPath(path_msg);
            path_msg.header.frame_id = frame_id_global_;
            path_msg.header.stamp = ros::Time::now();
            pub_path_.publish(path_msg);
        }
	}
}
