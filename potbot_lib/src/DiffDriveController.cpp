#include <potbot_lib/DiffDriveController.h>

namespace potbot_lib{

    void DiffDriveAgent::to_msg(nav_msgs::Odometry& odom_msg)
    {
        odom_msg.header.stamp           = ros::Time::now();
        odom_msg.pose.pose.position.x   = x;
        odom_msg.pose.pose.position.y   = y;
        odom_msg.pose.pose.position.z   = 0.0;
        odom_msg.pose.pose.orientation  = utility::get_Quat(0,0,yaw);
        odom_msg.twist.twist.linear.x   = v;
        odom_msg.twist.twist.linear.y   = 0.0;
        odom_msg.twist.twist.linear.z   = 0.0;
        odom_msg.twist.twist.angular.x  = 0.0;
        odom_msg.twist.twist.angular.y  = 0.0;
        odom_msg.twist.twist.angular.z  = omega;
    }

    void DiffDriveAgent::set_msg(const geometry_msgs::Pose& pose_msg)
    {
        x                               = pose_msg.position.x;
        y                               = pose_msg.position.y;
        yaw                             = utility::get_Yaw(pose_msg.orientation);
    }

    void DiffDriveAgent::set_msg(const geometry_msgs::PoseStamped& pose_msg)
    {
        set_msg(pose_msg.pose);
    }

    void DiffDriveAgent::set_msg(const nav_msgs::Odometry& odom_msg)
    {
        set_msg(odom_msg.pose.pose);
        v                               = odom_msg.twist.twist.linear.x;
        omega                           = odom_msg.twist.twist.angular.z;
    }

    void DiffDriveAgent::update()
    {
        yaw                             += omega*deltatime;
        x                               += v*deltatime*cos(yaw);
        y                               += v*deltatime*sin(yaw);
    }

    double DiffDriveAgent::get_distance(const Point& p)
    {
        return sqrt(pow(x-p.x,2) + pow(y-p.y,2));
    }

    double DiffDriveAgent::get_angle(const Point& p)
    {
        return atan2(p.y - y, p.x - x);
    }

    namespace Controller{

        void DiffDriveController::set_target(double x,double y,double yaw)
        {
            process_ = PROCESS_STOP;

            target_point_.x = x;
            target_point_.y = y;
            target_point_.theta = yaw;

            error_angle_i_ = 0.0;
            error_angle_pre_ = nan("");

            error_distance_i_ = 0.0;
            error_distance_pre_ = nan("");

            error_declination_i_ = 0.0;
            error_declination_pre_ = nan("");
        }

        void DiffDriveController::set_target(const geometry_msgs::Pose& pose_msg)
        {
            set_target(	pose_msg.position.x,
						pose_msg.position.y,
						potbot_lib::utility::get_Yaw(pose_msg.orientation));
        }

        void DiffDriveController::set_gain(double p,double i,double d)
        {
            gain_p_=p;
            gain_i_=i;
            gain_d_=d;
        }

        void DiffDriveController::set_time_state_gain(double k1, double k2)
        {
            time_state_k1_ = k1;
            time_state_k2_ = k2;
        }

        void DiffDriveController::set_margin(double angle, double distance)
        {
            stop_margin_angle_ = angle;
            stop_margin_distance_ = distance;
        }

        void DiffDriveController::set_limit(double linear, double angular)
        {
            max_linear_velocity_ = linear;
            max_angular_velocity_ = angular;
        }

        void DiffDriveController::set_distance_to_lookahead_point(double distance)
        {
            distance_to_lookahead_point_ = distance;
        }

        void DiffDriveController::set_target_path(const nav_msgs::Path& path_msg)
        {
            done_init_pose_alignment_ = false;
            set_init_pose_ = false;
            target_path_.clear();
            size_t idx = 0;
            target_path_index_ = 0;
            for (const auto pose : path_msg.poses)
            {
                Point p;
                p.index = idx++;
                p.x = pose.pose.position.x;
                p.y = pose.pose.position.y;
                p.theta = potbot_lib::utility::get_Yaw(pose.pose.orientation);
                target_path_.push_back(p);
            }
            lookahead_ = &target_path_.front();
        }

        void DiffDriveController::set_initialize_pose(bool ini)
        {
            initialize_pose_ = ini;
        }

        int DiffDriveController::get_current_process()
        {
            return process_;
        }

        int DiffDriveController::get_current_line_following_process()
        {
            return line_following_process_;
        }

        void DiffDriveController::get_lookahead(visualization_msgs::Marker& marker_msg)
        {
            
            if (lookahead_ == nullptr) return;

            marker_msg.ns                    = "LookAhead";
            marker_msg.id                    = 0;
            marker_msg.lifetime              = ros::Duration(0);

            marker_msg.type                  = visualization_msgs::Marker::SPHERE;
            marker_msg.action                = visualization_msgs::Marker::MODIFY;
            
            marker_msg.pose                  = potbot_lib::utility::get_Pose(lookahead_->x, lookahead_->y, 0, 0, 0, lookahead_->theta);

            marker_msg.scale.x               = 0.08;
            marker_msg.scale.y               = 0.08;
            marker_msg.scale.z               = 0.08;

            marker_msg.color                 = potbot_lib::color::get_msg(potbot_lib::color::RED);
            marker_msg.color.a               = 0.5;
            
        }

        double DiffDriveController::get_target_path_init_angle()
        {
            double init_angle = 0, x1 = 0, x2 = 1, y1 = 0, y2 = 0;
            if (target_path_index_ <= target_path_.size() - 2)
            {
                x1 = target_path_[target_path_index_].x;
                x2 = target_path_[target_path_index_+1].x;
                y1 = target_path_[target_path_index_].y;
                y2 = target_path_[target_path_index_+1].y;
            }
            else
            {
                x1 = target_path_[target_path_index_-1].x;
                x2 = target_path_[target_path_index_].x;
                y1 = target_path_[target_path_index_-1].y;
                y2 = target_path_[target_path_index_].y;
            }
            init_angle = atan2(y2-y1,x2-x1);
            return init_angle;
        }

        void DiffDriveController::apply_limit()
        {
            if (v > max_linear_velocity_) v = max_linear_velocity_;
            else if (v < -max_linear_velocity_) v = -max_linear_velocity_;
            if (omega > max_angular_velocity_) omega = max_angular_velocity_;
            else if (omega < -max_angular_velocity_) omega = -max_angular_velocity_; 
        }

        void DiffDriveController::pid_control_angle()
        {
            double error_angle = target_point_.theta-yaw;
            if (isfinite(error_angle_pre_)){
                error_angle_i_ += error_angle*deltatime;
                double error_angle_d = (error_angle - error_angle_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_angle + gain_i_*error_angle_i_ + gain_d_*error_angle_d;
                omega = alngular_velocity;
            }
            error_angle_pre_ = error_angle;
        }

        void DiffDriveController::pid_control_distance()
        {
            double error_distance = get_distance(target_point_);
            if (isfinite(error_distance_pre_)){
                error_distance_i_ += error_distance*deltatime;
                double error_distance_d = (error_distance - error_distance_pre_)/deltatime;
                double linear_velocity = gain_p_*error_distance + gain_i_*error_distance_i_ + gain_d_*error_distance_d;
                v = linear_velocity;
            }
            error_distance_pre_ = error_distance;
        }

        void DiffDriveController::pid_control_declination()
        {
            double error_declination = get_angle(target_point_)-yaw;
            if (isfinite(error_declination_pre_)){
                error_declination_i_ += error_declination*deltatime;
                double error_declination_d = (error_declination - error_declination_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_declination + gain_i_*error_declination_i_ + gain_d_*error_declination_d;
                omega = alngular_velocity;
            }
            error_declination_pre_ = error_declination;
        }

        void DiffDriveController::pid_control()
        {
            v=0;
            omega=0;
            
            if (process_ == PROCESS_STOP && (abs(target_point_.theta-yaw) >= stop_margin_angle_ || get_distance(target_point_) >= stop_margin_distance_))
            {
                process_ = PROCESS_ROTATE_DECLINATION;
            }

            if (process_ == PROCESS_ROTATE_DECLINATION && abs(get_angle(target_point_)-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STRAIGHT;
                error_declination_i_ = 0.0;
                error_declination_pre_ = nan("");
            }
            else if (process_ == PROCESS_STRAIGHT && get_distance(target_point_) < stop_margin_distance_)
            {
                process_ = PROCESS_ROTATE_ANGLE;
            }
            else if (process_ == PROCESS_ROTATE_ANGLE && abs(target_point_.theta-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STOP;
            }
            
            if (process_ == PROCESS_ROTATE_DECLINATION)
            {
                pid_control_declination();
            }
            else if (process_ == PROCESS_STRAIGHT)
            {
                pid_control_declination();
                pid_control_distance();
            }
            else if (process_ == PROCESS_ROTATE_ANGLE)
            {
                pid_control_angle();
            }

            apply_limit();
        }
        
        void DiffDriveController::pure_pursuit()
        {
            v=0;
            omega=0;

            if (target_path_.empty()) return;

            if (get_distance(target_path_.back()) <= distance_to_lookahead_point_)
            {
                line_following_process_ = PROCESS_STOP;
                return;
            }

            size_t target_path_size = target_path_.size();

            Point* sub_goal = &target_path_.front();
            double l_d;
            while(true)
            {
                sub_goal = &target_path_[target_path_index_];
                l_d = get_distance(*sub_goal);
                if (l_d <= distance_to_lookahead_point_)
                {
                    target_path_index_++;
                    if (target_path_index_ > target_path_size-1)
                    {
                        target_path_index_ = target_path_size-1;
                        return;
                        // break;
                    }
                }
                else
                {
                    lookahead_ = sub_goal;
                    break;
                }
            }

            ROS_INFO("process: %d, index: %d, size: %d", line_following_process_, target_path_index_, target_path_size);

            if(initialize_pose_ && !done_init_pose_alignment_)
            {
                double init_angle = get_target_path_init_angle();

                if (abs(init_angle - yaw) > stop_margin_angle_ || l_d > 2*distance_to_lookahead_point_)
                {
                    if (!set_init_pose_)
                    {
                        set_init_pose_ = true;
                        set_target(lookahead_->x, lookahead_->y, init_angle);
                        line_following_process_ = RETURN_TO_TARGET_PATH;
                    }
                }
                else
                {
                    line_following_process_ = FOLLOWING_PATH;
                }

                if (line_following_process_ == RETURN_TO_TARGET_PATH)
                {
                    pid_control();
                    apply_limit();
                }
                
            }
            else
            {
                line_following_process_ = FOLLOWING_PATH;
            }

            if (line_following_process_ == FOLLOWING_PATH && target_path_index_ < target_path_size)
            {   
                done_init_pose_alignment_ = true;
                double alpha = get_angle(*lookahead_) - yaw;
                v = max_linear_velocity_;
                omega = 2.0*v*sin(alpha)/l_d;
                apply_limit();
            } 
        }

        void DiffDriveController::time_state_control()
        {
            v = 0;
            omega = 0;
            if (abs(y) > stop_margin_distance_ || abs(yaw) > stop_margin_angle_)
            {
                line_following_process_ = TIME_STATE_CONTROL;
                v = max_linear_velocity_;
                double theta = yaw;
                // if (theta == M_PI_2) theta -= 0.01;
                // else if(theta == -M_PI_2) theta += 0.01;
                double z2 = tan(theta);
                double z3 = y;
                double mu2 = -time_state_k1_*z3 - time_state_k2_*z2;
                // omega = mu2*v*pow(cos(theta),3);
                omega = -time_state_k1_*z3*v - time_state_k2_*z2*abs(v);
            }
            else
            {
                line_following_process_ = PROCESS_STOP;
            }
        }

        void DiffDriveController::p166_41()
        {
            v = 0;
            omega = 0;
            if (abs(y) > stop_margin_distance_ || abs(yaw) > stop_margin_angle_)
            {
                line_following_process_ = TIME_STATE_CONTROL;   //TIME_STATE_CONTROL 変更
                double ky = 1;
                double kth = 1;
                v = max_linear_velocity_;
                omega = -ky*y -kth*yaw;
            }
            else
            {
                line_following_process_ = PROCESS_STOP;
            }
        }
    }
}