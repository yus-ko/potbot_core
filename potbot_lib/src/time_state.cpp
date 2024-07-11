#include <potbot_lib/time_state.h>

namespace potbot_lib{

    namespace controller{

        void TimeState::setTargetPath(const std::vector<Pose>& path)
        {
            target_path_ = path;
        }

        void TimeState::applyLimit()
        {
            v = std::min(v, max_linear_velocity_);
            v = std::max(v, -max_linear_velocity_);
            omega = std::min(omega, max_angular_velocity_);
            omega = std::max(omega, -max_angular_velocity_);
        }

        bool TimeState::reachedTarget()
        {
            if (target_path_.empty()) return true;
            return getDistance(target_path_.back()) <= stop_margin_distance_;
        }

        void TimeState::calculateCommand()
        {
            v = 0;
            omega = 0;
            if (1)
            {
                double ky = weight_y_;
                double kth = weight_yaw_;

                // v = max_linear_velocity_;
                // omega = -ky*y -kth*yaw;

                v = max_linear_velocity_;
                double theta = yaw;
                // if (theta == M_PI_2) theta -= 0.01;
                // else if(theta == -M_PI_2) theta += 0.01;
                double z2 = tan(theta);
                double z3 = y;
                double mu2 = -ky*z3 - kth*z2;
                omega = mu2*v*pow(cos(theta),3);
                // omega = -ky*z3*v - kth*z2*abs(v);
            }
        }
    }
}