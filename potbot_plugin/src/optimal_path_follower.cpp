#include <potbot_plugin/optimal_path_follower.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::OptimalPathFollower, potbot_base::Controller)
namespace potbot_nav
{
    namespace controller
    {

        void OptimalPathFollower::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
            pub_plans_ = private_nh.advertise<visualization_msgs::MarkerArray>("path/plans", 1);
            pub_best_plan_ = private_nh.advertise<nav_msgs::Path>("path/best", 1);
            pub_split_path_ = private_nh.advertise<nav_msgs::Path>("path/split", 1);
            pub_objective_function_ = private_nh.advertise<sensor_msgs::PointCloud2>("objective_function", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::OptimalPathFollowerConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::OptimalPathFollowerConfig>::CallbackType cb = boost::bind(&OptimalPathFollower::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void OptimalPathFollower::reconfigureCB(const potbot_plugin::OptimalPathFollowerConfig& param, uint32_t level)
        {
            optimizer_.setMargin(param.stop_margin_angle, param.stop_margin_distance);
            optimizer_.setLimit(param.min_linear_velocity, param.max_linear_velocity, param.min_angular_velocity, param.max_angular_velocity);
            optimizer_.setOptimizationMethod(param.optimization_method);
            optimizer_.setTimeIncrement(param.time_increment);
            optimizer_.setTimeEnd(param.time_end);
            optimizer_.setLinearVelocityIncrement(param.linear_velocity_increment);
            optimizer_.setAngularVelocityIncrement(param.angular_velocity_increment);
            optimizer_.setIterationMax(param.max_iteration);
            optimizer_.setLearningRate(param.learning_rate);
        }

        void OptimalPathFollower::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            potbot_lib::utility::to_agent(robot_pose_, optimizer_);
            if (reachedTarget()) return;
            optimizer_.calculateCommand();
            publishSplitPath();
            publishPlans();
            publishBestPlan();
            potbot_lib::utility::to_msg(optimizer_, robot_pose_);
            cmd_vel = robot_pose_.twist.twist;
        }

        void OptimalPathFollower::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            if (path_msg.empty()) return;
            target_path_ = path_msg;
            target_pose_ = path_msg.back();
            std::vector<potbot_lib::Pose> path;
            for (const auto pose : path_msg)
            {
                potbot_lib::Pose p;
                p.position.x = pose.pose.position.x;
                p.position.y = pose.pose.position.y;
                p.rotation.z = tf2::getYaw(pose.pose.orientation);
                path.push_back(p);
            }
            optimizer_.setTargetPath(path);
        }

        void OptimalPathFollower::getPlans(visualization_msgs::MarkerArray& msg)
        {
            msg.markers.clear();

            visualization_msgs::Marker path_msg;
            path_msg.header.frame_id = frame_id_global_;
            path_msg.id = 0;
            path_msg.lifetime = ros::Duration(0.1);
            path_msg.type = visualization_msgs::Marker::LINE_STRIP;
            path_msg.pose = potbot_lib::utility::get_pose();
            path_msg.color.a = 1;
            path_msg.color.g = 0.8;
            path_msg.scale.x = 0.001;

            std::vector<potbot_lib::controller::plan> plans;
            optimizer_.getPlans(plans);
            for (const auto& plan:plans)
            {
                path_msg.id++;
                path_msg.points.clear();
                for (const auto& p:plan.path)
                {
                    path_msg.points.push_back(potbot_lib::utility::get_point(p(0),p(1)));
                }
                msg.markers.push_back(path_msg);
            }
        }

        void OptimalPathFollower::getSplitPath(nav_msgs::Path& msg)
        {
            std::vector<Eigen::Vector2d> path;
            optimizer_.getSplitPath(path);
            potbot_lib::utility::to_msg(path, msg);
            msg.header.frame_id = frame_id_global_;
        }

        void OptimalPathFollower::getBestPath(nav_msgs::Path& msg)
        {
            std::vector<Eigen::Vector2d> path;
            optimizer_.getBestPath(path);
            potbot_lib::utility::to_msg(path, msg);
            msg.header.frame_id = frame_id_global_;
        }

        void OptimalPathFollower::getBestCmd(geometry_msgs::Twist& cmd)
        {
            optimizer_.getBestCmd(cmd.linear.x,cmd.angular.z);
        }

        void OptimalPathFollower::publishPlans()
        {
            visualization_msgs::MarkerArray plan_markers;
            getPlans(plan_markers);
            pub_plans_.publish(plan_markers);
        }

        void OptimalPathFollower::publishBestPlan()
        {
            nav_msgs::Path path_msg;
            getBestPath(path_msg);
            pub_best_plan_.publish(path_msg);
        }

        void OptimalPathFollower::publishSplitPath()
        {
            nav_msgs::Path path_msg;
            getSplitPath(path_msg);
            pub_split_path_.publish(path_msg);
        }

        bool OptimalPathFollower::reachedTarget()
        {
            return optimizer_.reachedTarget();
            // ROS_INFO_STREAM(abs(getDistance(target_point_)) << ":" << stop_margin_distance_);
            // return path_index_ == target_path_.size()-1 && abs(getDistance(target_point_)) <= stop_margin_distance_;
            // return abs(getDistance(target_point_)) <= stop_margin_distance_;
        }
    }
}