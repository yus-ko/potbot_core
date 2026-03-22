#include <potbot_lib/time_state.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace potbot_lib
{

    namespace controller
    {

        void TimeState::setTargetPath(const std::vector<Pose> &path)
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
            if (target_path_.empty())
            {
                return true;
            }
            return getDistance(target_path_.back()) <= stop_margin_distance_;
        }

        void TimeState::calculateCommand()
        {
            v = 0.0;
            omega = 0.0;

            if (target_path_.empty())
            {
                return;
            }

            size_t closest_idx = 0;
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < target_path_.size(); ++i)
            {
                double dx = target_path_[i].position.x - x;
                double dy = target_path_[i].position.y - y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = i;
                }
            }

            const Pose &target = target_path_[closest_idx];

            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);

            double dx = target.position.x - x;
            double dy = target.position.y - y;

            double error_y = -sin_yaw * dx + cos_yaw * dy;
            double error_yaw = std::remainder(target.rotation.z - yaw, 2.0 * M_PI);

            v = max_linear_velocity_;
            omega = weight_y_ * error_y + weight_yaw_ * error_yaw;

            applyLimit();
        }

    }
}
