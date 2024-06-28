#ifndef H_DIFFDRIVECONTROLLER_
#define H_DIFFDRIVECONTROLLER_

#include <potbot_lib/diff_drive_agent.h>
#include <potbot_lib/utility_ros.h>

namespace potbot_lib{

    namespace controller{
        
        const int PROCESS_STOP = 0;
        const int PROCESS_ROTATE_DECLINATION = 1;
        const int PROCESS_STRAIGHT = 2;
        const int PROCESS_ROTATE_ANGLE = 3;
        const int RETURN_TO_TARGET_PATH = 4;
        const int FOLLOWING_PATH = 5;
        const int TIME_STATE_CONTROL = 6;

        class DiffDriveController : public DiffDriveAgent{
            protected:
                Pose target_point_;

                std::vector<Pose> target_path_;
                Pose* lookahead_ = &target_path_.front();
                size_t target_path_index_ = 0;
                double distance_to_lookahead_point_ = 0.3;

                double gain_p_ = 1.0;
                double gain_i_ = 0.5;
                double gain_d_ = 0.001;

                double time_state_k1_ = 2.0;
                double time_state_k2_ = 1.0;

                double stop_margin_angle_ = 0.1;
                double stop_margin_distance_ = 0.03;

                double max_linear_velocity_ = 1.0;
                double max_angular_velocity_ = M_PI;

                double error_angle_i_ = 0.0;
                double error_angle_pre_ = nan("");

                double error_distance_i_ = 0.0;
                double error_distance_pre_ = nan("");

                double error_declination_i_ = 0.0;
                double error_declination_pre_ = nan("");

                int process_ = PROCESS_STOP;
                int line_following_process_ = PROCESS_STOP;
                
                bool done_init_pose_alignment_ = false;
                bool initialize_pose_ = true;
                bool set_init_pose_ = false;

            public:
                DiffDriveController();
                ~DiffDriveController(){};

                void initPID();

                void setTarget(double x, double y, double yaw);
                void setTarget(Pose target);
                void setTargetPath(const std::vector<Pose>& path);
                void setGain(double p, double i, double d);
                void setTimeStateGain(double k1, double k2);
                void setMargin(double angle, double distance);
                void setLimit(double linear, double angular);
                void setDistanceToLookaheadPoint(double distance);
                void setInitializePose(bool ini);

                int getCurrentProcess();
                int getCurrentLineFollowingProcess();
                Pose getLookahead();
                double getTargetPathInitAngle();

                void applyLimit();

                void pidControlAngle();
                void pidControlDistance();
                void pidControlDeclination();
                void pidControl();
                
                void timeStateControl();
                void p166_41();

                bool reachedTarget();
            
        };
    }
}

#endif	// H_DIFFDRIVECONTROLLER_