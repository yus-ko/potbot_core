#ifndef H_POTBOT_DYNAMIC_WINDOW_APPROACH_
#define H_POTBOT_DYNAMIC_WINDOW_APPROACH_

#include <potbot_lib/base_controller.h>
#include <potbot_lib/diff_drive_controller.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/DWAConfig.h>

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

        class DynamicWindowApproach : public BaseController
        {
            private:
                ros::Publisher pub_plans_, pub_best_plan_, pub_split_path_;
                std::string frame_id_global_ = "map";
                bool reset_path_index_ = true;
                int path_index_ = 0;
                dynamic_reconfigure::Server<potbot_lib::DWAConfig> *dsrv_;

                void reconfigureCB(const potbot_lib::DWAConfig& param, uint32_t level);

                double time_increment_ = 0.1;
                double time_end_ = 1.0;
                double linear_velocity_min_ = -0.2;
                double linear_velocity_max_ = 0.2;
                double linear_velocity_increment_ = 0.05;
                double angular_velocity_min_ = -1.0;
                double angular_velocity_max_ = 1.0;
                double angular_velocity_increment_ = 0.1;

                std::vector<Eigen::Vector2d> target_path_;
                std::vector<Eigen::Vector2d> split_path_;

                std::vector<plan> plans_;
                plan best_plan_;

                int closest_index_pre_ = 0;

                void splitPath();
                void searchForBestPlan();

                void createPlans(); 

                void getPlans(std::vector<plan>& plans);
                void getPlans(visualization_msgs::MarkerArray& msg);
                void getSplitPath(std::vector<Eigen::Vector2d>& path);
                void getSplitPath(nav_msgs::Path& msg);
                void getBestPath(std::vector<Eigen::Vector2d>& path);
                void getBestPath(nav_msgs::Path& msg);
                void getBestPlan(plan& plan);
                void getBestCmd(double& v, double& omega);
                void getBestCmd(geometry_msgs::Twist& cmd);

                void publishPlans();
                void publishBestPlan();
                void publishSplitPath();

                bool reachedTarget();

            public:
                DynamicWindowApproach(){};
                ~DynamicWindowApproach(){};

                void initialize(std::string name);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);
        };
    }
}

#endif	// H_POTBOT_DYNAMIC_WINDOW_APPROACH_