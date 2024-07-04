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

            dsrv_ = new dynamic_reconfigure::Server<potbot_lib::DWAConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_lib::DWAConfig>::CallbackType cb = boost::bind(&DynamicWindowApproach::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void DynamicWindowApproach::reconfigureCB(const potbot_lib::DWAConfig& param, uint32_t level)
        {
            setMargin(	param.stop_margin_angle, param.stop_margin_distance);
            setLimit( param.max_linear_velocity, param.max_angular_velocity);
            reset_path_index_ = param.reset_path_index;
            time_increment_ = param.time_increment;
            time_end_ = param.time_end;
            linear_velocity_min_ = param.min_linear_velocity;
            linear_velocity_max_ = param.max_linear_velocity;
            linear_velocity_increment_ = param.linear_velocity_increment;
            angular_velocity_min_ = param.min_angular_velocity;
            angular_velocity_max_ = param.max_angular_velocity;
            angular_velocity_increment_ = param.angular_velocity_increment;
            learning_rate_ = param.learning_rate;
        }

        void DynamicWindowApproach::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            ROS_DEBUG("path index: %d / %d", (int)path_index_, (int)target_path_.size());
            if (reachedTarget() || target_path_.empty()) return;
            
            // createPlans();
            splitPath();
            // searchForBestPlan();
            searchForBestPlanWithGradient();
            publishPlans();
            publishBestPlan();
            publishSplitPath();
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
            // applyLimit();
            nav_msgs::Odometry robot;
            toMsg(robot);
            cmd_vel = robot.twist.twist;
        }

        void DynamicWindowApproach::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            target_path_.clear();
            for (const auto& p:path_msg)
            {
                target_path_.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
            }

            Pose g;
            if (!target_path_.empty())
            {
                g.position.x = target_path_.back()[0];
                g.position.y = target_path_.back()[1];
            }
            target_point_ = g;

            if (reset_path_index_) 
            {
                Eigen::Vector2d closest_point;
                utility::find_closest_vector(target_path_, Eigen::Vector2d(x,y), closest_point);
                path_index_ = utility::get_index(target_path_, closest_point);
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

            for (size_t i = path_index_; i < target_path_.size(); i++)
            {
                if ((target_path_[i] - Eigen::Vector2d(x,y)).norm() < stop_margin_distance_)
                {
                    path_index_=i;
                }
                else
                {
                    break;
                }
            }

            size_t closest_index = 0;
            double mind = 1e100;
            for (size_t i = 0; i < target_path_.size(); i++)
            {
                double d = (target_path_[i] - Eigen::Vector2d(x,y)).norm();
                if (d < mind)
                {
                    closest_index = i;
                    mind = d;
                }
            }

            // closest_index = path_index_;
            double v = std::max(abs(linear_velocity_min_),abs(linear_velocity_max_));
            double total_distance = 0;
            double limit_ditance = v*time_end_;
            double inc_distance = v*time_increment_;
            double downsample_distance = 0; 
            int shift = 0;
            // if (target_path_.size() - closest_index > 5) shift = 5;
            for (int i = closest_index+shift; i < target_path_.size(); i++)
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

        void DynamicWindowApproach::searchForBestPlanWithGradient()
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

            double v = v;
            double omega = omega;

            double score_min = 1e100;
            double score_min_threshold = 1e-2;
            double learning_rate = learning_rate_;
            size_t iteration_max = 100;

            for (size_t i = 0; i < iteration_max; i++)
            {
                plan p;
                p.linear_velocity = v;
                p.angular_velocity = omega;
                p.delta_time = dt;
                p.end_time = tau;
                size_t num_points = split_path_.size();
                Eigen::Vector2d gradient(0,0);
                // for (double t = 0; t < tau; t += dt)
                for (int j = 0; j < num_points; j++)
                {
                    double t = dt*j;
                    double th = th_init + omega*t;
                    double x = x_init + v*t*cos(th);
                    double y = y_init + v*t*sin(th);
                    p.path.push_back(Eigen::Vector2d(x,y));
                
                    double x2 = split_path_[j](0);
                    double y2 = split_path_[j](1);

                    double s1 = x - x2;
                    double s2 = y - y2;
                    double s3 = sqrt(pow(s1,2)+pow(s2,2));

                    double gv = t*(cos(th)*s1+sin(th)*s2)/(s3); 
                    // double gv =
                    //     2*t*sin(th)*(t*sin(th)*v-y2+y_init) + 2*t*cos(th)*(t*cos(th)*v-x2+x_init)/
                    //     2*sqrt(pow(t*sin(th)*v-y2+y_init,2)+pow(t*cos(th)-x2+x_init,2));

                    double gomega = -t*t*v*(sin(th)*s1-cos(th)*s2)/(s3);
                    // double gomega = 
                    //     t*t*v*((x2-x_init)*sin(th)+(y_init-y2)*cos(th))/
                    //     sqrt(pow(t*v*sin(th)-y2+y_init,2)+pow(t*v*cos(th)-x2+x_init,2));

                    gradient+=Eigen::Vector2d(gv,gomega);
                }

                double sum = 0;
                for (size_t j = 0; j < num_points; j++)
                {
                    sum += (p.path[j] - split_path_[j]).norm();
                }

                plans_.push_back(p);
                
                if (sum < score_min)
                {
                    best_plan_ = p;
                    score_min = sum;
                    if (gradient.norm() < score_min_threshold)
                    {
                        break;
                    }
                }

                v -= learning_rate * gradient(0);
                omega -= learning_rate * gradient(1);

                v = std::min(v,v_max);
                v = std::max(v,v_min);
                omega = std::min(omega,omega_max);
                omega = std::max(omega,omega_min);
    
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

        bool DynamicWindowApproach::reachedTarget()
        {
            // ROS_INFO_STREAM(abs(getDistance(target_point_)) << ":" << stop_margin_distance_);
            // return path_index_ == target_path_.size()-1 && abs(getDistance(target_point_)) <= stop_margin_distance_;
            return abs(getDistance(target_point_)) <= stop_margin_distance_;
        }
    }
}