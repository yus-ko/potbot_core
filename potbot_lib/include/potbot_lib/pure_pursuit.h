#ifndef H_POTBOT_LIB_PUREPURSUIT_
#define H_POTBOT_LIB_PUREPURSUIT_

#include <potbot_lib/diff_drive_agent.h>

namespace potbot_lib{

    namespace controller{

        class PurePursuit : public DiffDriveAgent{
            protected:
                std::vector<Pose> target_path_;
                Pose* lookahead_ = &target_path_.front();
                double distance_to_lookahead_point_ = 0.3;

                double stop_margin_angle_ = 0.1;
                double stop_margin_distance_ = 0.03;

                double max_linear_velocity_ = 1.0;
                double max_angular_velocity_ = M_PI;

            public:
                PurePursuit(){};
                ~PurePursuit(){};

                void setTargetPath(const std::vector<Pose>& path);
                void setMargin(double angle, double distance);
                void setLimit(double linear, double angular);
                void setDistanceToLookaheadPoint(double distance);

                Pose getLookahead();

                void applyLimit();

                bool reachedTarget();

                void calculateCommand();
            
        };
    }
}

#endif	// H_POTBOT_LIB_PUREPURSUIT_