#include <potbot_lib/particle_swarm_optimization.h>

namespace potbot_lib
{
    namespace path_planner
	{
		ParticleSwarmOptimization::ParticleSwarmOptimization(): generator_(0), random_(0.0, 1.0)
		{
			random_ = std::uniform_real_distribution<double>(-1, 1);
			initialize();
		}

		void ParticleSwarmOptimization::initialize()
		{
			// std::random_device random_device;        		// ハードウェア乱数生成器
			// generator_.seed(random_device());
			generator_.seed(0);
			size_t params = 2;
			size_t particle_num = particle_num_;

			particles_.resize(params, particle_num);
			velocities_.resize(params, particle_num);
			pbest_.resize(params+1, particle_num);
			for (size_t i = 0; i < particle_num; i++)
			{
				Eigen::VectorXd particle_vec(params);
				particle_vec << robot_.position.x, robot_.position.y;
				Eigen::VectorXd velocity_vec(params);
				for (size_t j = 0; j < params; j++) 
				{
					velocity_vec(j) = random_(generator_);
				}
				particles_.col(i) << particle_vec;
				velocities_.col(i) << velocity_vec;
				pbest_.col(i) << particle_vec , objective_function(particles_.col(i));
				// particle_vec.push_back(Point(particles.col(i)(0), particles.col(1), z));
				// velocity_vec.push_back(Point(dist_delta(eng), dist_delta(eng), dist_delta(eng)));
			}

		}

		void ParticleSwarmOptimization::setCalculationParam(int pnum, int max_itr)
		{
			particle_num_ = pnum;
			max_iteration_ = max_itr;
		}

		void ParticleSwarmOptimization::setWeight(double wv, double wp, double wg)
		{
			weight_velocity_ = wv;
			weight_pbest_ = wp;
			weight_gbest_ = wg;
		}

		void ParticleSwarmOptimization::setThresholdDistance(double d)
		{
			threshold_distance_to_obstacle_ = d;
		}

		void ParticleSwarmOptimization::createPath()
		{
			gbest_history_.clear();
			size_t params = particles_.rows();
			size_t particle_num = particles_.cols();
			size_t max_iter = max_iteration_;
			double w = weight_velocity_;
			double c1 = weight_pbest_;
			double c2 = weight_gbest_;

			// Point pbest = find_min_z(particle_vec);
			Eigen::VectorXd gbest = get_min_value_column(pbest_);
			// gbest = Point(g_x_max,g_y_max,objective_function(g_x_max,g_y_max));
			gbest_history_.push_back(gbest);

			particles_history_.clear();
			for (size_t i = 0; i < particles_.cols(); i++)
			{
				std::vector<Eigen::VectorXd> vecs;
				vecs.push_back(particles_.col(i));
				particles_history_.push_back(vecs);
			}

			size_t iter = 0;
			while (iter < max_iter)
			{
				// ROS_INFO("%d / %d", int(iter), int(max_iter));
				for (size_t i = 0; i < particle_num; i++)
				{
					particles_.col(i) = particles_.col(i) + velocities_.col(i);
					Eigen::VectorXd pb = pbest_.block(0, i, pbest_.rows()-1, 1) - particles_.col(i).head(params);
					Eigen::VectorXd gb = gbest.block(0, 0, gbest.rows()-1, 1) - particles_.col(i).head(params);
					velocities_.col(i) = w*velocities_.col(i) + c1*random_(generator_)*(pb) + c2*random_(generator_)*(gb);
					// std::cout << velocities.col(i) << std::endl;

					double pbest_value = pbest_(params,i);
					double i_value = objective_function(particles_.col(i));
					if (i_value < pbest_value)
					{
						pbest_.col(i) << particles_.col(i) , i_value;
					}
					
				}
				gbest = get_min_value_column(pbest_);
				for (size_t i = 0; i < particles_.cols(); i++)
				{
					particles_history_[i].push_back(particles_.col(i));
				}
				gbest_history_.push_back(gbest);

				iter++;
				// std::cout << "Answer: " << gbest.transpose() << std::endl;

				// visualization_msgs::Marker particles_marker_msg;
				// mat_to_marker_points(particles, particles_marker_msg);
				// particles_marker_msg.header.frame_id = "map";

				// visualization_msgs::Marker optimized_marker_msg;
				// point_to_marker(gbest, optimized_marker_msg);
				// optimized_marker_msg.header.frame_id = "map";
				
				// pub_particles_.publish(particles_marker_msg);
				// pub_optimized_.publish(optimized_marker_msg);
			}
		}

		Eigen::VectorXd ParticleSwarmOptimization::get_min_value_column(const Eigen::MatrixXd& mat) {
			// 3行目の要素を取得
			Eigen::VectorXd thirdRow = mat.row(mat.rows()-1);
			// std::cout << thirdRow << std::endl;

			// 3行目の最小値のインデックスを取得
			Eigen::Index minIndex;
			double minValue = thirdRow.minCoeff(&minIndex);
			// std::cout<< std::endl << minIndex << ":" << minValue << std::endl;

			// 最小値の列を取得
			Eigen::VectorXd minValueColumn = mat.col(minIndex);

			return minValueColumn;
		}

		double ParticleSwarmOptimization::ackley_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double b_sum = 0;
			double d_sum = 0;
			for (size_t i = 0; i < n; i++)
			{
				b_sum += pow(vec(i),2);
				d_sum += cos(2*M_PI*vec(i));
			}
			
			double a = 20;
			double b = -20*exp(-0.2*sqrt(1.0/double(n)*b_sum));
			double c = M_E;
			double d = -exp(1.0/double(n)*d_sum);
			return a + b + c + d;
		}

		double ParticleSwarmOptimization::rastrigin_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double b_sum = 0;
			for (size_t i = 0; i < n; i++) b_sum += (pow(vec(i),2) - 10.0*cos(2*M_PI*vec(i)));
			double a = 10.0 * double(n);
			double b = b_sum;
			return a + b;
		}

		double ParticleSwarmOptimization::griewank_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double b_sum = 0;
			double c_sum = 1.0;
			for (size_t i = 0; i < n; i++)
			{
				b_sum += pow(vec(i),2);
				c_sum *= cos(vec(i)/sqrt(double(i)+1.0));
			}
			
			double a = 1.0;
			double b = 1.0/4000.0*b_sum;
			double c = -c_sum;
			return a + b + c;
		}

		double ParticleSwarmOptimization::styblinski_tang_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double a_sum = 0;
			for (size_t i = 0; i < n; i++)
			{
				a_sum += pow(vec(i),4) - 16.0*pow(vec(i),2) + 5.0*vec(i);
			}
			
			double a = a_sum/2.0;
			return a;
		}

		double ParticleSwarmOptimization::michalewicz_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double m = 10.0;
			double a_sum = 0;
			for (size_t i = 0; i < n; i++)
			{
				a_sum += sin(vec(i))*pow(sin((double(i)+1.0)*pow(vec(i),2)/M_PI),m);
			}
			double a = -a_sum;
			return a;
		}

		double ParticleSwarmOptimization::xin_she_yang_function(const Eigen::VectorXd& vec)
		{
			size_t n = vec.size();
			double a_sum = 0;
			double b_sum = 0;
			for (size_t i = 0; i < n; i++)
			{
				a_sum += abs(vec(i));
				b_sum += sin(pow(vec(i),2));
			}
			double a = a_sum*exp(-b_sum);
			return a;
		}

		double ParticleSwarmOptimization::objective_function(const Eigen::VectorXd& vec)
		{
			// return ackley_function(vec);
			// return rastrigin_function(vec);
			// return griewank_function(vec);
			// return styblinski_tang_function(vec);
			// return michalewicz_function(vec);
			// return xin_she_yang_function(vec);

			Point p = Point(vec(0), vec(1));
			double distance_to_goal = (p - goal_.position).norm();
			for (const auto& o:obstacles_)
			{
				double distance_to_obstacle = (p - o).norm();
				if (distance_to_obstacle < threshold_distance_to_obstacle_)
				{
					// return 1e100;
					return distance_to_goal + 1/(distance_to_obstacle+1e-100);
				}
			}
			return distance_to_goal;
		}

		double ParticleSwarmOptimization::objective_function(double x, double y)
		{
			Eigen::VectorXd vec(2);
			vec << x,y;
			return objective_function(vec);
		}

		void ParticleSwarmOptimization::createSurface(std::vector<Point>& points_output, double time)
		{
			double x_min = -5.0 + robot_.position.x;
			double x_max = 5.0 + robot_.position.x;
			double y_min = -5.0 + robot_.position.y;
			double y_max = 5.0 + robot_.position.y;
			double resolution = 0.1;

			points_output.clear();
			for (double x = x_min; x <= x_max; x+=resolution)
			{
				for (double y = y_min; y <= y_max; y+=resolution)
				{
					// double px = x + 11*sin(time);
					// double py = y + 11*cos(time);
					double px = x;
					double py = y;
					double pz = objective_function(px,py);
					points_output.push_back(Point(px,py,pz));
				}
			}

			// for (double r = 0; r <= sqrt(200); r+=g_resolution)
			// {
			// 	for (double th = 0; th <= 2*M_PI; th+=g_resolution)
			// 	{
			// 		double x = r*cos(th);
			// 		double y = r*sin(th);
					
			// 		double px = x + 11*sin(time);
			// 		double py = y + 11*cos(time);
			// 		double pz = objective_function(px,py);
			// 		points_output.push_back(Point(px,py,pz));
			// 	}
			// }
		}

		void ParticleSwarmOptimization::setGoal(const Pose& vec)
		{
			goal_ = vec;
		}

		void ParticleSwarmOptimization::setRobot(const Pose& vec)
		{
			robot_ = vec;
		}

		void ParticleSwarmOptimization::setObstacle(const Point& vec)
		{
			obstacles_.push_back(vec);
		}

		void ParticleSwarmOptimization::clearObstacles()
		{
			obstacles_.clear();
		}

		void ParticleSwarmOptimization::getPath(std::vector<Pose>& path)
        {
            potbot_lib::utility::vec_to_path(gbest_history_, path);
			std::vector<Pose> path_raw = path;
			potbot_lib::utility::bezier(path_raw, path);
        }

		void ParticleSwarmOptimization::getBestHistory(std::vector<Eigen::VectorXd>& path)
		{
			path = gbest_history_;
		}

		void ParticleSwarmOptimization::getParticlesHistory(std::vector<std::vector<Eigen::VectorXd>>& path)
		{
			path = particles_history_;
		}
	}
}
