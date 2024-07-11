#include <potbot_lib/pure_pursuit.h>

namespace potbot_lib{

    namespace controller{

        void PurePursuit::setTargetPath(const std::vector<Pose>& path)
        {
            target_path_ = path;
            lookahead_ = &target_path_.front();
        }

        Pose PurePursuit::getLookahead()
        {
            if (lookahead_ == nullptr) return Pose{};
            else return *lookahead_;
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
            if (target_path_.empty()) return true;
            return getDistance(target_path_.back()) <= stop_margin_distance_;
        }

        void PurePursuit::calculateCommand()
        {
            v=0;
            omega=0;

            size_t target_path_index = 0;

            if (target_path_.empty()) return;

            size_t target_path_size = target_path_.size();

            double d_min = 1e100;
            for (size_t i = 0; i < target_path_size; i++)
            {
                double d = getDistance(target_path_[i]);
                if (d < d_min)
                {
                    d_min = d;
                    target_path_index = i;
                }
            }

            double d_sum = 0;
            bool got_lookahead = false;
            for (size_t i = target_path_index; i < target_path_size; i++)
            {
                if (i > 0)
                {
                    d_sum += hypot(target_path_[i].position.x - target_path_[i-1].position.x,target_path_[i].position.y - target_path_[i-1].position.y);
                    if (d_sum > distance_to_lookahead_point_)
                    {
                        lookahead_ = &target_path_[i];
                        got_lookahead = true;
                        break;
                    }
                }
            }

            if (!got_lookahead)
            {
                v = max_linear_velocity_;
                return;
            }
            

            double l_d = getDistance(*lookahead_);

            double alpha = getAngle(*lookahead_) - yaw;
            if (normalize_)
            {
                v = max_linear_velocity_/(abs(alpha)+1.0);
                double nv = v/max_linear_velocity_;
                omega = 2.0*nv*sin(alpha)/l_d;
            }
            else
            {
                v = max_linear_velocity_;
                omega = 2.0*v*sin(alpha)/l_d;
            }
            
            applyLimit();
        }
    }
}