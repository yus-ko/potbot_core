#ifndef HPP_POTBOT_LIB_PURE_PURSUIT_
#define HPP_POTBOT_LIB_PURE_PURSUIT_

#include <potbot_lib/diff_drive_agent.hpp>

#include <cmath>
#include <vector>

namespace potbot_lib
{

    namespace controller
    {

        class PurePursuit : public DiffDriveAgent
        {

        protected:
            std::vector<Pose> target_path_;
            double distance_to_lookahead_point_ = 0.3;
            double stop_margin_angle_ = 0.1;
            double stop_margin_distance_ = 0.03;
            double max_linear_velocity_ = 1.0;
            double max_angular_velocity_ = M_PI;
            bool normalize_ = true;

        public:
            PurePursuit() = default;
            ~PurePursuit() = default;

            void setTargetPath(const std::vector<Pose> &path);
            void setMargin(double angle, double distance)
            {
                stop_margin_angle_ = angle;
                stop_margin_distance_ = distance;
            };
            void setLimit(double linear, double angular)
            {
                max_linear_velocity_ = linear;
                max_angular_velocity_ = angular;
            };
            void setDistanceToLookaheadPoint(double val) { distance_to_lookahead_point_ = val; };
            void setNormalize(bool val) { normalize_ = val; };

            Pose getLookahead();
            void applyLimit();
            bool reachedTarget();
            void calculateCommand();
        };

    }
}

#endif // HPP_POTBOT_LIB_PURE_PURSUIT_
