#include <potbot_lib/diff_drive_controller.h>

namespace potbot_lib{

    namespace controller{
        
        DiffDriveController::DiffDriveController()
        {
            initPID();
        }

        void DiffDriveController::initPID()
        {
            process_ = PROCESS_STOP;
            error_angle_i_ = 0.0;
            error_angle_pre_ = nan("");

            error_distance_i_ = 0.0;
            error_distance_pre_ = nan("");

            error_declination_i_ = 0.0;
            error_declination_pre_ = nan("");
        }

        void DiffDriveController::setTarget(double x,double y,double yaw)
        {
            target_point_.position.x = x;
            target_point_.position.y = y;
            target_point_.rotation.z = yaw;
        }

        void DiffDriveController::setTarget(Pose target)
        {
            target_point_ = target;
        }

        void DiffDriveController::setTargetPath(const std::vector<Pose>& path)
        {
            done_init_pose_alignment_ = false;
            set_init_pose_ = false;
            target_path_.clear();
            size_t idx = 0;
            target_path_index_ = 0;
            target_path_ = path;
            setTarget(target_path_.back());
            lookahead_ = &target_path_.front();
        }

        void DiffDriveController::setGain(double p,double i,double d)
        {
            gain_p_=p;
            gain_i_=i;
            gain_d_=d;
        }

        void DiffDriveController::setTimeStateGain(double k1, double k2)
        {
            time_state_k1_ = k1;
            time_state_k2_ = k2;
        }

        void DiffDriveController::setMargin(double angle, double distance)
        {
            stop_margin_angle_ = angle;
            stop_margin_distance_ = distance;
        }

        void DiffDriveController::setLimit(double linear, double angular)
        {
            max_linear_velocity_ = linear;
            max_angular_velocity_ = angular;
        }

        void DiffDriveController::setDistanceToLookaheadPoint(double distance)
        {
            distance_to_lookahead_point_ = distance;
        }

        void DiffDriveController::setInitializePose(bool ini)
        {
            initialize_pose_ = ini;
        }

        int DiffDriveController::getCurrentProcess()
        {
            return process_;
        }

        int DiffDriveController::getCurrentLineFollowingProcess()
        {
            return line_following_process_;
        }

        Pose DiffDriveController::getLookahead()
        {
            if (lookahead_ == nullptr) return Pose{};
            else return *lookahead_;
        }

        double DiffDriveController::getTargetPathInitAngle()
        {
            double init_angle = 0, x1 = 0, x2 = 1, y1 = 0, y2 = 0;
            if (target_path_index_ <= target_path_.size() - 2)
            {
                x1 = target_path_[target_path_index_].position.x;
                x2 = target_path_[target_path_index_+1].position.x;
                y1 = target_path_[target_path_index_].position.y;
                y2 = target_path_[target_path_index_+1].position.y;
            }
            else
            {
                x1 = target_path_[target_path_index_-1].position.x;
                x2 = target_path_[target_path_index_].position.x;
                y1 = target_path_[target_path_index_-1].position.y;
                y2 = target_path_[target_path_index_].position.y;
            }
            init_angle = atan2(y2-y1,x2-x1);
            return init_angle;
        }

        void DiffDriveController::applyLimit()
        {
            if (v > max_linear_velocity_) v = max_linear_velocity_;
            else if (v < -max_linear_velocity_) v = -max_linear_velocity_;
            if (omega > max_angular_velocity_) omega = max_angular_velocity_;
            else if (omega < -max_angular_velocity_) omega = -max_angular_velocity_; 
        }

        void DiffDriveController::pidControlAngle()
        {
            // ROS_INFO_STREAM();
            double error_angle = target_point_.rotation.z-yaw;
            if (isfinite(error_angle_pre_)){
                error_angle_i_ += error_angle*deltatime;
                double error_angle_d = (error_angle - error_angle_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_angle + gain_i_*error_angle_i_ + gain_d_*error_angle_d;
                omega = alngular_velocity;
            }
            error_angle_pre_ = error_angle;
        }

        void DiffDriveController::pidControlDistance()
        {
            double error_distance = getDistance(target_point_);
            if (isfinite(error_distance_pre_)){
                error_distance_i_ += error_distance*deltatime;
                double error_distance_d = (error_distance - error_distance_pre_)/deltatime;
                double linear_velocity = gain_p_*error_distance + gain_i_*error_distance_i_ + gain_d_*error_distance_d;
                v = linear_velocity;
            }
            error_distance_pre_ = error_distance;
        }

        void DiffDriveController::pidControlDeclination()
        {
            double error_declination = getAngle(target_point_)-yaw;
            if (isfinite(error_declination_pre_)){
                error_declination_i_ += error_declination*deltatime;
                double error_declination_d = (error_declination - error_declination_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_declination + gain_i_*error_declination_i_ + gain_d_*error_declination_d;
                omega = alngular_velocity;
            }
            error_declination_pre_ = error_declination;
        }

        void DiffDriveController::pidControl()
        {
            v=0;
            omega=0;
            
            if (process_ == PROCESS_STOP && (abs(target_point_.rotation.z-yaw) >= stop_margin_angle_ || getDistance(target_point_) >= stop_margin_distance_))
            {
                process_ = PROCESS_ROTATE_DECLINATION;
            }

            if (process_ == PROCESS_ROTATE_DECLINATION && abs(getAngle(target_point_)-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STRAIGHT;
                error_declination_i_ = 0.0;
                error_declination_pre_ = nan("");
            }
            else if (process_ == PROCESS_STRAIGHT && getDistance(target_point_) < stop_margin_distance_)
            {
                process_ = PROCESS_ROTATE_ANGLE;
            }
            else if (process_ == PROCESS_ROTATE_ANGLE && abs(target_point_.rotation.z-yaw) < stop_margin_angle_)
            {
                process_ = PROCESS_STOP;
            }
            
            if (process_ == PROCESS_ROTATE_DECLINATION)
            {
                pidControlDeclination();
            }
            else if (process_ == PROCESS_STRAIGHT)
            {
                pidControlDeclination();
                pidControlDistance();
            }
            else if (process_ == PROCESS_ROTATE_ANGLE)
            {
                pidControlAngle();
            }

            applyLimit();
            ROS_DEBUG("pid: v:%f omega:%f", v, omega);
        }

        void DiffDriveController::timeStateControl()
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

        bool DiffDriveController::reachedTarget()
        {
            return abs(getDistance(target_point_)) <= stop_margin_distance_ && abs(getAngle(target_point_)) <= stop_margin_angle_;
        }
    }
}