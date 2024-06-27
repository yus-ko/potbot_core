#include <potbot_lib/dynamic_window_approach.h>

PLUGINLIB_EXPORT_CLASS(potbot_lib::controller::DynamicWindowApproach, potbot_lib::controller::BaseController)
namespace potbot_lib
{
    namespace controller
    {

        void DynamicWindowApproach::initialize(std::string name)
        {
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
            pub_plans_ = private_nh.advertise<visualization_msgs::MarkerArray>("path/plans", 1);
            pub_best_plan_ = private_nh.advertise<nav_msgs::Path>("path/best", 1);
            pub_split_path_ = private_nh.advertise<nav_msgs::Path>("path/split", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_lib::ControllerConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_lib::ControllerConfig>::CallbackType cb = boost::bind(&DynamicWindowApproach::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void DynamicWindowApproach::reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level)
        {
            setGain( param.gain_p, param.gain_i, param.gain_d);
            setMargin(	param.stop_margin_angle, param.stop_margin_distance);
            setLimit( param.max_linear_velocity, param.max_linear_velocity);
            setDistanceToLookaheadPoint(param.distance_to_lookahead_point);
            reset_path_index_ = param.reset_path_index;
        }

        void DynamicWindowApproach::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            createPlans();
            splitPath();
            searchForBestPlan();
            publishPlans();
            publishBestPlan();
            publishSplitPath();
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
            nav_msgs::Odometry robot;
            toMsg(robot);
            cmd_vel = robot.twist.twist;
        }

        void DynamicWindowApproach::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            closest_index_pre_ = 0;
            target_path_.clear();
            for (const auto& p:path_msg)
            {
                target_path_.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
            }
        }

        void DynamicWindowApproach::getPlans(std::vector<plan>& plans)
        {
            plans = plans_;
        }

        void DynamicWindowApproach::getPlans(visualization_msgs::MarkerArray& msg)
        {
            msg.markers.clear();

            visualization_msgs::Marker path_msg;
            path_msg.header.frame_id = frame_id_global_;
            path_msg.id = 0;
            path_msg.lifetime = ros::Duration(0.1);
            path_msg.type = visualization_msgs::Marker::LINE_STRIP;
            path_msg.pose = utility::get_Pose();
            path_msg.color.a = 1;
            path_msg.color.g = 0.8;
            path_msg.scale.x = 0.001;

            for (const auto& plan:plans_)
            {
                path_msg.id++;
                path_msg.points.clear();
                for (const auto& p:plan.path)
                {
                    path_msg.points.push_back(utility::get_Point(p(0),p(1)));
                }
                msg.markers.push_back(path_msg);
            }
        }

        void DynamicWindowApproach::getSplitPath(std::vector<Eigen::Vector2d>& path)
        {
            path = split_path_;
        }

        void DynamicWindowApproach::getSplitPath(nav_msgs::Path& msg)
        {
            utility::to_msg(split_path_, msg);
            msg.header.frame_id = frame_id_global_;
        }

        void DynamicWindowApproach::getBestPath(std::vector<Eigen::Vector2d>& path)
        {
            path = best_plan_.path;
        }

        void DynamicWindowApproach::getBestPath(nav_msgs::Path& msg)
        {
            utility::to_msg(best_plan_.path, msg);
            msg.header.frame_id = frame_id_global_;
        }

        void DynamicWindowApproach::getBestPlan(plan& plan)
        {
            plan = best_plan_;
        }

        void DynamicWindowApproach::getBestCmd(double& v, double& omega)
        {
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
        }

        void DynamicWindowApproach::getBestCmd(geometry_msgs::Twist& cmd)
        {
            cmd.linear.x = best_plan_.linear_velocity;
            cmd.angular.z = best_plan_.angular_velocity;
        }

        void DynamicWindowApproach::splitPath()
        {
            split_path_.clear();
            Eigen::Vector2d closest_point;
            utility::find_closest_vector(target_path_, Eigen::Vector2d(x,y), closest_point);
            // int closest_index = 0;
            int closest_index = utility::get_index(target_path_, closest_point);
            if (closest_index < closest_index_pre_) closest_index = closest_index_pre_+1;
            closest_index_pre_ = closest_index;

            double total_distance = 0;
            double limit_ditance = max_linear_velocity_*time_end_;
            double inc_distance = max_linear_velocity_*time_increment_;
            double downsample_distance = 0; 
            for (int i = closest_index; i < target_path_.size(); i++)
            {
                if (i < 2)
                {
                    split_path_.push_back(target_path_[i]);
                    continue;
                }
                double distance = (target_path_[i] - target_path_[i-1]).norm();
                total_distance += distance;
                if (total_distance < limit_ditance)
                {
                    if (downsample_distance == 0) split_path_.push_back(target_path_[i]);
                    
                }
                else
                {
                    break;
                }
                downsample_distance += distance;
                if (downsample_distance >= inc_distance) downsample_distance = 0;
            }
        }

        void DynamicWindowApproach::searchForBestPlan()
        {
            // plan tmp;
            // best_plan_ = tmp;
            double score_min = 1e9;
            for (const auto& plan:plans_)
            {
                size_t num_points = plan.path.size();
                if (num_points >= split_path_.size()) num_points = split_path_.size();

                double score_diff = 0;
                for (size_t i = 0; i < num_points; i++) 
                {
                    score_diff += (plan.path[i] - split_path_[i]).norm();
                }

                if (score_diff < score_min)
                {
                    best_plan_ = plan;
                    score_min = score_diff;
                }
            }
        }

        void DynamicWindowApproach::createPlans()
        {
            plans_.clear();
            double x_init = x;
            double y_init = y;
            double th_init = yaw;
            double dt = time_increment_;
            double tau = time_end_;
            double v_min = linear_velocity_min_;
            double v_max = linear_velocity_max_;
            double v_delta = linear_velocity_increment_;
            double omega_min = angular_velocity_min_;
            double omega_max = angular_velocity_max_;
            double omega_delta = angular_velocity_increment_;
            
            for (double v = v_min; v <= v_max; v += v_delta)
            {
                for (double omega = omega_min; omega <= omega_max; omega += omega_delta)
                {
                    plan p;
                    p.linear_velocity = v;
                    p.angular_velocity = omega;
                    p.delta_time = dt;
                    p.end_time = tau;
                    double x=x_init, y=y_init, th=th_init;
                    for (double t = 0; t < tau; t += dt)
                    {
                        th += omega*dt;
                        x += v*dt*cos(th);
                        y += v*dt*sin(th);
                        p.path.push_back(Eigen::Vector2d(x,y));
                    }
                    plans_.push_back(p);
                }
            }
        }

        void DynamicWindowApproach::publishPlans()
        {
            visualization_msgs::MarkerArray plan_markers;
            getPlans(plan_markers);
            pub_plans_.publish(plan_markers);
        }

        void DynamicWindowApproach::publishBestPlan()
        {
            nav_msgs::Path path_msg;
            getBestPath(path_msg);
            pub_best_plan_.publish(path_msg);
        }

        void DynamicWindowApproach::publishSplitPath()
        {
            nav_msgs::Path path_msg;
            getSplitPath(path_msg);
            pub_split_path_.publish(path_msg);
        }
    }
}