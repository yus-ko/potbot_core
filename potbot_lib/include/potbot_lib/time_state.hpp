#ifndef HPP_POTBOT_LIB_TIME_STATE_
#define HPP_POTBOT_LIB_TIME_STATE_

#include <potbot_lib/diff_drive_agent.hpp>

#include <cmath>
#include <vector>

namespace potbot_lib
{

    namespace controller
    {

        class TimeState : public DiffDriveAgent
        {

        protected:
            std::vector<Pose> target_path_;
            double weight_y_ = 1.0;
            double weight_yaw_ = 1.0;
            double stop_margin_angle_ = 0.1;
            double stop_margin_distance_ = 0.03;
            double max_linear_velocity_ = 1.0;
            double max_angular_velocity_ = M_PI;

        public:
            TimeState() = default;
            ~TimeState() = default;

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
            void setWeight(double ky, double kyaw)
            {
                weight_y_ = ky;
                weight_yaw_ = kyaw;
            };

            void applyLimit();
            bool reachedTarget();
            void calculateCommand();
        };

    }
}

#endif // HPP_POTBOT_LIB_TIME_STATE_
