#include <potbot_lib/pid.h>

namespace potbot_lib{

    namespace controller{

        void PID::applyLimit()
        {
            v = std::min(v, max_linear_velocity_);
            v = std::max(v, -max_linear_velocity_);
            omega = std::min(omega, max_angular_velocity_);
            omega = std::max(omega, -max_angular_velocity_);
        }

        bool PID::reachedTarget()
        {
            return getDistance(target_point_) <= stop_margin_distance_ && abs(getAngle(target_point_)) <= stop_margin_angle_;
        }

        void PID::calculateCommand()
        {
            pidControl();
        }

        void PID::initPID()
        {
            process_ = PROCESS_STOP;
            error_angle_i_ = 0.0;
            error_angle_pre_ = nan("");

            error_distance_i_ = 0.0;
            error_distance_pre_ = nan("");

            error_declination_i_ = 0.0;
            error_declination_pre_ = nan("");
        }

        void PID::setTargetPoint(const Pose& target)
        {
            initPID();
            target_point_ = target;
        }

        void PID::pidControlAngle()
        {
            // ROS_INFO_STREAM();
            double error_angle = target_point_.rotation.z-yaw;
            if (std::isfinite(error_angle_pre_)){
                error_angle_i_ += error_angle*deltatime;
                double error_angle_d = (error_angle - error_angle_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_angle + gain_i_*error_angle_i_ + gain_d_*error_angle_d;
                omega = alngular_velocity;
            }
            error_angle_pre_ = error_angle;
        }

        void PID::pidControlDistance()
        {
            double error_distance = getDistance(target_point_);
            if (std::isfinite(error_distance_pre_)){
                error_distance_i_ += error_distance*deltatime;
                double error_distance_d = (error_distance - error_distance_pre_)/deltatime;
                double linear_velocity = gain_p_*error_distance + gain_i_*error_distance_i_ + gain_d_*error_distance_d;
                v = linear_velocity;
            }
            error_distance_pre_ = error_distance;
        }

        void PID::pidControlDeclination()
        {
            double error_declination = getAngle(target_point_)-yaw;
            if (std::isfinite(error_declination_pre_)){
                error_declination_i_ += error_declination*deltatime;
                double error_declination_d = (error_declination - error_declination_pre_)/deltatime;
                double alngular_velocity = gain_p_*error_declination + gain_i_*error_declination_i_ + gain_d_*error_declination_d;
                omega = alngular_velocity;
            }
            error_declination_pre_ = error_declination;
        }

        void PID::pidControl()
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
        }

    }
}