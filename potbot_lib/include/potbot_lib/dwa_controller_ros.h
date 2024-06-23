#ifndef H_DWACONTROLLER_ROS_
#define H_DWACONTROLLER_ROS_

#include <potbot_lib/diff_drive_controller.h>
#include <visualization_msgs/MarkerArray.h>

namespace potbot_lib
{

    namespace controller
    {

        typedef struct
        {
            std::vector<Eigen::Vector2d> path;
            double linear_velocity=0;
            double angular_velocity=0;
            double end_time=0;
            double delta_time=0;
        } plan;

        class DWAControllerROS : public DiffDriveController
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

                void splitPath();
                void searchForBestPlan();

                void createPlans();

            public:
                DWAControllerROS(){};
                ~DWAControllerROS(){};

                void setTimeIncrement(const double param) {time_increment_ = param;};
                void setTimeEnd(const double param) {time_end_ = param;};
                void setLinearVelocityMin(const double param) {linear_velocity_min_ = param;};
                void setLinearVelocityMax(const double param) {linear_velocity_max_ = param;};
                void setLinearVelocityIncrement(const double param) {linear_velocity_increment_ = param;};
                void setAngularVelocityMin(const double param) {angular_velocity_min_ = param;};
                void setAngularVelocityMax(const double param) {angular_velocity_max_ = param;};
                void setAngularVelocityIncrement(const double param) {angular_velocity_increment_ = param;};

                void setDwaTargetPath(const nav_msgs::Path& msg);

                void getPlans(std::vector<plan>& plans);
                void getPlans(visualization_msgs::MarkerArray& msg);
                void getSplitPath(std::vector<Eigen::Vector2d>& path);
                void getSplitPath(nav_msgs::Path& msg);
                void getBestPath(std::vector<Eigen::Vector2d>& path);
                void getBestPath(nav_msgs::Path& msg);
                void getBestPlan(plan& plan);
                void getBestCmd(double& v, double& omega);
                void getBestCmd(geometry_msgs::Twist& cmd);

                void calculateCommand();
            
        };
    }
}

#endif	// H_DWACONTROLLER_ROS_