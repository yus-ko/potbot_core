#include <potbot_lib/pure_pursuit.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace potbot_lib
{

    namespace controller
    {

        void PurePursuit::setTargetPath(const std::vector<Pose> &path)
        {
            target_path_ = path;
        }

        Pose PurePursuit::getLookahead()
        {
            if (target_path_.empty())
            {
                return Pose(x, y, 0.0);
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

            double accumulated = 0.0;
            size_t lookahead_idx = closest_idx;
            for (size_t i = closest_idx; i + 1 < target_path_.size(); ++i)
            {
                double dx = target_path_[i + 1].position.x - target_path_[i].position.x;
                double dy = target_path_[i + 1].position.y - target_path_[i].position.y;
                accumulated += std::sqrt(dx * dx + dy * dy);
                lookahead_idx = i + 1;
                if (accumulated >= distance_to_lookahead_point_)
                {
                    break;
                }
            }

            return target_path_[lookahead_idx];
        }

        void PurePursuit::applyLimit()
        {
            v = std::min(v, max_linear_velocity_);
            v = std::max(v, -max_linear_velocity_);
            omega = std::min(omega, max_angular_velocity_);
            omega = std::max(omega, -max_angular_velocity_);
        }

        bool PurePursuit::reachedTarget()
        {
            if (target_path_.empty())
            {
                return true;
            }
            return getDistance(target_path_.back()) <= stop_margin_distance_;
        }

        void PurePursuit::calculateCommand()
        {
            v = 0.0;
            omega = 0.0;

            if (target_path_.empty())
            {
                return;
            }

            Pose lookahead = getLookahead();

            double dx = lookahead.position.x - x;
            double dy = lookahead.position.y - y;

            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);

            double distance_lateral = -sin_yaw * dx + cos_yaw * dy;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 1e-6)
            {
                return;
            }

            v = max_linear_velocity_;
            omega = 2.0 * v * distance_lateral / (distance * distance);

            applyLimit();
        }

    }
}
