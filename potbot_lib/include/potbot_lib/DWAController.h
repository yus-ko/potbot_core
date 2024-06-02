#ifndef _H_DWACONTROLLER_
#define _H_DWACONTROLLER_

#include <potbot_lib/DiffDriveController.h>
#include <visualization_msgs/MarkerArray.h>

namespace potbot_lib
{

    namespace Controller
    {

        typedef struct
        {
            std::vector<Eigen::Vector2d> path;
            double linear_velocity=0;
            double angular_velocity=0;
            double end_time=0;
            double delta_time=0;
        } plan;

        class DWAController : public DiffDriveController
        {
            protected:
                double time_increment_ = 0.1;
                double time_end_ = 1.0;
                double linear_velocity_min_ = -0.2;
                double linear_velocity_max_ = 0.2;
                double linear_velocity_increment_ = 0.05;
                double angular_velocity_min_ = -1.0;
                double angular_velocity_max_ = 1.0;
                double angular_velocity_increment_ = 0.1;

                std::vector<Eigen::Vector2d> dwa_target_path_;
                std::vector<Eigen::Vector2d> split_path_;

                std::vector<plan> plans_;
                plan best_plan_;

                int closest_index_pre = 0;

                void __split_path();
                void __search_for_best_plan();

                void __create_plans();

            public:
                DWAController(){};
                ~DWAController(){};

                void set_time_increment(const double param) {time_increment_ = param;};
                void set_time_end(const double param) {time_end_ = param;};
                void set_linear_velocity_min(const double param) {linear_velocity_min_ = param;};
                void set_linear_velocity_max(const double param) {linear_velocity_max_ = param;};
                void set_linear_velocity_increment(const double param) {linear_velocity_increment_ = param;};
                void set_angular_velocity_min(const double param) {angular_velocity_min_ = param;};
                void set_angular_velocity_max(const double param) {angular_velocity_max_ = param;};
                void set_angular_velocity_increment(const double param) {angular_velocity_increment_ = param;};

                void set_dwa_target_path(const nav_msgs::Path& msg);

                void get_plans(std::vector<plan>& plans);
                void get_plans(visualization_msgs::MarkerArray& msg);
                void get_split_path(std::vector<Eigen::Vector2d>& path);
                void get_split_path(nav_msgs::Path& msg);
                void get_best_path(std::vector<Eigen::Vector2d>& path);
                void get_best_path(nav_msgs::Path& msg);
                void get_best_plan(plan& plan);
                void get_best_cmd(double& v, double& omega);
                void get_best_cmd(geometry_msgs::Twist& cmd);

                void calculate_command();
            
        };
    }
}

#endif	// _H_DWACONTROLLER_