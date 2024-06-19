#ifndef _H_DIFFDRIVECONTROLLER_
#define _H_DIFFDRIVECONTROLLER_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <potbot_lib/Utility.h>

namespace potbot_lib{

    class DiffDriveAgent{
        public:
            double x                            = 0.0;  //x軸位置 [m]
            double y                            = 0.0;  //y軸位置 [m]
            double yaw                          = 0.0;  //z軸回転 [rad]
            double v                            = 0.0;  //並進速度 [m/s]
            double omega                        = 0.0;  //回転角速度 [rad/s]

            double deltatime                    = 0.02; //単位時間 [s]

            DiffDriveAgent( const double x              = 0.0,
                            const double y              = 0.0,
                            const double yaw            = 0.0,
                            const double v              = 0.0,
                            const double omega          = 0.0,
                            const double deltatime      = 0.02):
                            x(x),
                            y(y),
                            yaw(yaw),
                            v(v),
                            omega(omega),
                            deltatime(deltatime){};

            void to_msg(nav_msgs::Odometry& odom_msg);
            void set_msg(const geometry_msgs::Pose& pose_msg);
            void set_msg(const geometry_msgs::PoseStamped& pose_msg);
            void set_msg(const nav_msgs::Odometry& odom_msg);
            void update();

            double get_distance(const Point& p);
            double get_angle(const Point& p);
    };

    namespace Controller{
        
        const int PROCESS_STOP = 0;
        const int PROCESS_ROTATE_DECLINATION = 1;
        const int PROCESS_STRAIGHT = 2;
        const int PROCESS_ROTATE_ANGLE = 3;
        const int RETURN_TO_TARGET_PATH = 4;
        const int FOLLOWING_PATH = 5;
        const int TIME_STATE_CONTROL = 6;

        class DiffDriveController : public DiffDriveAgent{
            protected:
                Point target_point_;

                std::vector<Point> target_path_;
                Point* lookahead_ = &target_path_.front();
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
                DiffDriveController(){};
                ~DiffDriveController(){};

                void set_target(double x, double y, double yaw);
                void set_target(const geometry_msgs::Pose& pose_msg);
                void set_gain(double p, double i, double d);
                void set_time_state_gain(double k1, double k2);
                void set_margin(double angle, double distance);
                void set_limit(double linear, double angular);
                void set_distance_to_lookahead_point(double distance);
                void set_target_path(const std::vector<geometry_msgs::PoseStamped>& path_msg);
                void set_target_path(const nav_msgs::Path& path_msg);
                void set_initialize_pose(bool ini);

                int get_current_process();
                int get_current_line_following_process();
                void get_lookahead(visualization_msgs::Marker& marker_msg);
                double get_target_path_init_angle();

                void apply_limit();

                void pid_control_angle();
                void pid_control_distance();
                void pid_control_declination();
                void pid_control();

                void pure_pursuit();
                void normalized_pure_pursuit();
                
                void time_state_control();
                void p166_41();
            
        };
    }
}

#endif	// _H_DIFFDRIVECONTROLLER_