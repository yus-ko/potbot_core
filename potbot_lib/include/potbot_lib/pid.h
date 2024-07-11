#ifndef H_POTBOT_LIB_POSITION_PID_
#define H_POTBOT_LIB_POSITION_PID_

#include <potbot_lib/diff_drive_agent.h>

namespace potbot_lib{

    namespace controller{

        const int PROCESS_STOP = 0;
        const int PROCESS_ROTATE_DECLINATION = 1;
        const int PROCESS_STRAIGHT = 2;
        const int PROCESS_ROTATE_ANGLE = 3;

        class PID : public DiffDriveAgent{

            protected:
                Pose target_point_;

                double gain_p_ = 1.0;
                double gain_i_ = 0.5;
                double gain_d_ = 0.001;

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

                void pidControlAngle();
                void pidControlDistance();
                void pidControlDeclination();
                void pidControl();

            public:
                PID(){};
                ~PID(){};

                void setTargetPoint(const Pose& target);
                void setMargin(double angle, double distance){
                    stop_margin_angle_ = angle;
                    stop_margin_distance_ = distance;};
                void setLimit(double linear, double angular){
                    max_linear_velocity_ = linear;
                    max_angular_velocity_ = angular;
                };
                void setGain(double p, double i, double d){
                    gain_p_ = p;
                    gain_i_ = i;
                    gain_d_ = d;
                };

                void initPID();
                
                int getCurrentProcess() {return process_;};

                void applyLimit();

                bool reachedTarget();

                void calculateCommand();
            
        };
    }
}

#endif	// H_POTBOT_LIB_POSITION_PID_