#ifndef H_PSO_PATH_PLANNER_
#define H_PSO_PATH_PLANNER_

#include <iostream>
#include <random>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <potbot_lib/utility.h>

namespace potbot_lib
{
    namespace path_planner
	{
        class ParticleSwarmOptimization
		{
			protected:
				std::mt19937 generator_;         				// メルセンヌ・ツイスタ乱数エンジン
				std::uniform_real_distribution<double> random_;	// 一様分布

				std::vector<Point> obstacles_;
				Pose robot_;
				Pose goal_;
				// std::vector<Pose> path_;

				int particle_num_ = 100;
				int max_iteration_ = 1000;
				double weight_velocity_ = 0.6;
				double weight_pbest_ = 0.25;
				double weight_gbest_ = 0.25;
				double threshold_distance_to_obstacle_ = 0.3;

				Eigen::MatrixXd particles_;
				Eigen::MatrixXd velocities_;
				Eigen::MatrixXd pbest_;

				std::vector<Eigen::VectorXd> gbest_history_;
				std::vector<std::vector<Eigen::VectorXd>> particles_history_;

				Eigen::VectorXd get_min_value_column(const Eigen::MatrixXd& mat);
				double ackley_function(const Eigen::VectorXd& vec);
				double rastrigin_function(const Eigen::VectorXd& vec);
				double griewank_function(const Eigen::VectorXd& vec);
				double styblinski_tang_function(const Eigen::VectorXd& vec);
				double michalewicz_function(const Eigen::VectorXd& vec);
				double xin_she_yang_function(const Eigen::VectorXd& vec);
				double objective_function(const Eigen::VectorXd& vec);
				double objective_function(double x, double y);

			public:
				ParticleSwarmOptimization();
                ~ParticleSwarmOptimization(){};

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
		};
	}
}

#endif	// H_PSO_PATH_PLANNER_
