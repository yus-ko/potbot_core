#include <potbot_lib/dwa_controller_ros.h>

namespace potbot_lib
{

    namespace controller
    {
        
        void DWAControllerROS::setDwaTargetPath(const nav_msgs::Path& msg)
        {
            closest_index_pre = 0;
            dwa_target_path_.clear();
            for (const auto& p:msg.poses)
            {
                dwa_target_path_.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
            }
            
        }

        void DWAControllerROS::getPlans(std::vector<plan>& plans)
        {
            plans = plans_;
        }

        void DWAControllerROS::getPlans(visualization_msgs::MarkerArray& msg)
        {
            msg.markers.clear();

            visualization_msgs::Marker path_msg;
            path_msg.header.frame_id="map";
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

        void DWAControllerROS::getSplitPath(std::vector<Eigen::Vector2d>& path)
        {
            path = split_path_;
        }

        void DWAControllerROS::getSplitPath(nav_msgs::Path& msg)
        {
            utility::to_msg(split_path_, msg);
        }

        void DWAControllerROS::getBestPath(std::vector<Eigen::Vector2d>& path)
        {
            path = best_plan_.path;
        }

        void DWAControllerROS::getBestPath(nav_msgs::Path& msg)
        {
            utility::to_msg(best_plan_.path, msg);
        }

        void DWAControllerROS::getBestPlan(plan& plan)
        {
            plan = best_plan_;
        }

        void DWAControllerROS::getBestCmd(double& v, double& omega)
        {
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
        }

        void DWAControllerROS::getBestCmd(geometry_msgs::Twist& cmd)
        {
            cmd.linear.x = best_plan_.linear_velocity;
            cmd.angular.z = best_plan_.angular_velocity;
        }

        void DWAControllerROS::splitPath()
        {
            split_path_.clear();
            Eigen::Vector2d closest_point;
            utility::find_closest_vector(dwa_target_path_, Eigen::Vector2d(x,y), closest_point);
            // int closest_index = 0;
            int closest_index = utility::get_index(dwa_target_path_, closest_point);
            if (closest_index < closest_index_pre) closest_index = closest_index_pre+1;
            closest_index_pre = closest_index;

            double total_distance = 0;
            double limit_ditance = max_linear_velocity_*time_end_;
            double inc_distance = max_linear_velocity_*time_increment_;
            double downsample_distance = 0; 
            for (int i = closest_index; i < dwa_target_path_.size(); i++)
            {
                if (i < 2)
                {
                    split_path_.push_back(dwa_target_path_[i]);
                    continue;
                }
                double distance = (dwa_target_path_[i] - dwa_target_path_[i-1]).norm();
                total_distance += distance;
                if (total_distance < limit_ditance)
                {
                    if (downsample_distance == 0) split_path_.push_back(dwa_target_path_[i]);
                    
                }
                else
                {
                    break;
                }
                downsample_distance += distance;
                if (downsample_distance >= inc_distance) downsample_distance = 0;
            }
        }

        void DWAControllerROS::searchForBestPlan()
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

        void DWAControllerROS::createPlans()
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

        void DWAControllerROS::calculateCommand()
        {
            createPlans();
            splitPath();
            searchForBestPlan();
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
        }
    }
}