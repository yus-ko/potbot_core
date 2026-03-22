#ifndef HPP_POTBOT_LIB_PARTICLE_SWARM_OPTIMIZATION_
#define HPP_POTBOT_LIB_PARTICLE_SWARM_OPTIMIZATION_

#include <potbot_lib/utility.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>

namespace potbot_lib
{

    class ParticleSwarmOptimization
    {
    private:
        std::vector<Point> obstacles_;
        Pose robot_;
        Pose goal_;

        int particle_num_  = 100;
        int max_iteration_ = 1000;
        double weight_velocity_ = 0.6;
        double weight_pbest_    = 0.25;
        double weight_gbest_    = 0.25;
        double threshold_distance_to_obstacle_ = 0.3;

        std::vector<Eigen::VectorXd> gbest_history_;
        std::vector<std::vector<Eigen::VectorXd>> particles_history_;

        std::vector<Pose> path_;

        double ackley_function(const Eigen::VectorXd& vec);
        double rastrigin_function(const Eigen::VectorXd& vec);
        double griewank_function(const Eigen::VectorXd& vec);
        double styblinski_tang_function(const Eigen::VectorXd& vec);
        double michalewicz_function(const Eigen::VectorXd& vec);
        double xin_she_yang_function(const Eigen::VectorXd& vec);

    public:
        ParticleSwarmOptimization();
        ~ParticleSwarmOptimization() {}

        void initialize();
        void createPath();
        void createSurface(std::vector<Point>& points_output, double time = 0);

        void setGoal(const Pose& vec);
        void setRobot(const Pose& vec);
        void setObstacle(const Point& vec);
        void setCalculationParam(int pnum, int max_itr);
        void setWeight(double wv, double wp, double wg);
        void setThresholdDistance(double d);
        void clearObstacles();

        void getPath(std::vector<Pose>& path);
        void getBestHistory(std::vector<Eigen::VectorXd>& path);
        void getParticlesHistory(std::vector<std::vector<Eigen::VectorXd>>& path);

        double objective_function(const Eigen::VectorXd& vec);
        double objective_function(double x, double y);
    };

} // namespace potbot_lib

#endif // HPP_POTBOT_LIB_PARTICLE_SWARM_OPTIMIZATION_
