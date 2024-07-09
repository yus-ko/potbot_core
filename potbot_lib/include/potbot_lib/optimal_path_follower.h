#ifndef H_POTBOT_LIB_OPTIMAL_PATH_FOLLOWER_
#define H_POTBOT_LIB_OPTIMAL_PATH_FOLLOWER_

#include <potbot_lib/diff_drive_agent.h>
#include <algorithm>

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

        class OptimalPathFollower : public DiffDriveAgent
        {
            private:
                std::string optimization_method_ = "gradient";
                double time_increment_ = 0.1;
                double time_end_ = 1.0;
                double linear_velocity_min_ = -0.2;
                double linear_velocity_max_ = 0.2;
                double linear_velocity_increment_ = 0.05;
                double angular_velocity_min_ = -1.0;
                double angular_velocity_max_ = 1.0;
                double angular_velocity_increment_ = 0.1;
                size_t iteration_max_ = 100;
                double learning_rate_ = 0.1;

                std::vector<Eigen::Vector2d> target_path_;
                std::vector<Eigen::Vector2d> split_path_;

                std::vector<plan> plans_;
                plan best_plan_;

                int closest_index_pre_ = 0;

                void splitPath();
                void searchForBestPlan();

                void searchForBestPlanWithGradient();

                void createPlans(); 
                
            public:
                OptimalPathFollower(){};
                ~OptimalPathFollower(){};

                void setOptimizationMethod(std::string val){optimization_method_ = val;};
                void setTimeIncrement(double val){time_increment_ = val;};
                void setTimeEnd(double val){time_end_ = val;};
                void setLinearVelocityMin(double val){linear_velocity_min_ = val;};
                void setLinearVelocityMax(double val){linear_velocity_max_ = val;};
                void setLinearVelocityIncrement(double val){linear_velocity_increment_ = val;};
                void setAngularVelocityMin(double val){angular_velocity_min_ = val;};
                void setAngularVelocityMax(double val){angular_velocity_max_ = val;};
                void setAngularVelocityIncrement(double val){angular_velocity_increment_ = val;};
                void setIterationMax(size_t val){iteration_max_ = val;};
                void setLearningRate(double val){learning_rate_ = val;};

                void getPlans(std::vector<plan>& plans);
                void getSplitPath(std::vector<Eigen::Vector2d>& path);
                void getBestPath(std::vector<Eigen::Vector2d>& path);
                void getBestPlan(plan& plan);
                void getBestCmd(double& v, double& omega);
                
                void calculateCommand();
                void setTargetPath(const std::vector<Pose>& path);
                bool reachedTarget();
        };
    }
}

#endif	// H_POTBOT_LIB_OPTIMAL_PATH_FOLLOWER_