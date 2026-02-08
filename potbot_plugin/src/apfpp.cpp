#include <potbot_plugin/apfpp.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::path_planner::APFPP, potbot_base::PathPlanner)
namespace potbot_nav
{
    namespace path_planner
    {
        void APFPP::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            apf_ = new potbot_lib::ArtificialPotentialFieldROS(name + "/apf");
            apfpp_ = new potbot_lib::path_planner::APFPathPlannerROS(name + "/apfpp", apf_);
            tf_ = tf;
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
			// pub_path_ = private_nh.advertise<nav_msgs::Path>("path", 1);
			// pub_surface_ = private_nh.advertise<sensor_msgs::PointCloud2>("surface", 1);
			// pub_optimized_ = private_nh.advertise<visualization_msgs::Marker>("optimized", 1);
			// pub_particles_ = private_nh.advertise<visualization_msgs::Marker>("particles", 1);
			// pub_debug_marker_ = private_nh.advertise<visualization_msgs::MarkerArray>("debug_markers", 1);

            // dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig>(private_nh);
            // dynamic_reconfigure::Server<potbot_plugin::ParticleSwarmOptimizationConfig>::CallbackType cb = boost::bind(&ParticleSwarmOptimization::reconfigureCB, this, _1, _2);
            // dsrv_->setCallback(cb);
        }

        // void APFPP::reconfigureCB(const potbot_plugin::ParticleSwarmOptimizationConfig& param, uint32_t level)
        // {
        //     // apfpp_.setCalculationParam(param.particle_num, param.max_iteration);
		// 	// apfpp_.setWeight(param.weight_velocity, param.weight_pbest, param.weight_gbest);
		// 	// apfpp_.setThresholdDistance(param.threshold_distance_to_obstacle);
        // }

        void APFPP::getPath(std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            apfpp_->getPath(path_msg);
        }

        void APFPP::planPath()
        {
            apf_->initPotentialField();
            apf_->getApf()->initField(
                apf_->getApf()->getHeader().rows, 
                apf_->getApf()->getHeader().cols, 
                apf_->getApf()->getHeader().resolution, 
                robot_pose_.pose.pose.position.x, 
                robot_pose_.pose.pose.position.y);
            frame_id_global_ = robot_pose_.header.frame_id;
            apf_->setFrameIdGlobal(robot_pose_.header.frame_id);
			apf_->setRobot(robot_pose_.pose.pose);
			apf_->setGoal(target_pose_);
			apf_->clearObstacles();
			apf_->setObstacle(obstacles_);

			apf_->createPotentialField();
			apf_->publishPotentialField();

			std::vector<std::vector<double>> path_raw, path_interpolated;
			double init_yaw = tf2::getYaw(robot_pose_.pose.pose.orientation);
			
			if (isnan(init_yaw)) init_yaw = 0;

            apfpp_->createPath(init_yaw);
            // apfpp_->createPathWithWeight(init_yaw);

            apfpp_->publishRawPath();
			apfpp_->bezier();
			apfpp_->publishPath();
        }
    }
}