#include <potbot_lib/particle_swarm_optimization.hpp>

#include <cmath>
#include <limits>

namespace potbot_lib
{

    ParticleSwarmOptimization::ParticleSwarmOptimization()
    {
    }

    void ParticleSwarmOptimization::initialize()
    {
        gbest_history_.clear();
        particles_history_.clear();
        path_.clear();
    }

    double ParticleSwarmOptimization::ackley_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        double sum_sq  = 0.0;
        double sum_cos = 0.0;
        for (int i = 0; i < n; ++i) {
            sum_sq  += vec[i] * vec[i];
            sum_cos += std::cos(2.0 * M_PI * vec[i]);
        }
        return -20.0 * std::exp(-0.2 * std::sqrt(sum_sq / n))
               - std::exp(sum_cos / n)
               + 20.0 + M_E;
    }

    double ParticleSwarmOptimization::rastrigin_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        double val = 10.0 * n;
        for (int i = 0; i < n; ++i) {
            val += vec[i] * vec[i] - 10.0 * std::cos(2.0 * M_PI * vec[i]);
        }
        return val;
    }

    double ParticleSwarmOptimization::griewank_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        double sum = 0.0;
        double prod = 1.0;
        for (int i = 0; i < n; ++i) {
            sum  += vec[i] * vec[i] / 4000.0;
            prod *= std::cos(vec[i] / std::sqrt(static_cast<double>(i + 1)));
        }
        return sum - prod + 1.0;
    }

    double ParticleSwarmOptimization::styblinski_tang_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        double val = 0.0;
        for (int i = 0; i < n; ++i) {
            double xi = vec[i];
            val += std::pow(xi, 4) - 16.0 * xi * xi + 5.0 * xi;
        }
        return val / 2.0;
    }

    double ParticleSwarmOptimization::michalewicz_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        const int m = 10;
        double val = 0.0;
        for (int i = 0; i < n; ++i) {
            val -= std::sin(vec[i]) *
                   std::pow(std::sin(static_cast<double>(i + 1) * vec[i] * vec[i] / M_PI),
                            2.0 * m);
        }
        return val;
    }

    double ParticleSwarmOptimization::xin_she_yang_function(const Eigen::VectorXd& vec)
    {
        const int n = static_cast<int>(vec.size());
        double sum1 = 0.0;
        double sum2 = 0.0;
        for (int i = 0; i < n; ++i) {
            sum1 += std::abs(vec[i]);
            sum2 += std::sin(vec[i] * vec[i]);
        }
        return sum1 * std::exp(-sum2);
    }

    double ParticleSwarmOptimization::objective_function(const Eigen::VectorXd& vec)
    {
        return objective_function(vec[0], vec[1]);
    }

    double ParticleSwarmOptimization::objective_function(double x, double y)
    {
        double dx_goal = goal_.position.x - x;
        double dy_goal = goal_.position.y - y;
        double dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

        double repulsion = 0.0;
        for (const auto& obs : obstacles_) {
            double dx = obs.x - x;
            double dy = obs.y - y;
            double d  = std::sqrt(dx * dx + dy * dy);
            if (d < threshold_distance_to_obstacle_) {
                repulsion += 1.0 / (d + 1e-6);
            }
        }

        return dist_goal + repulsion;
    }

    void ParticleSwarmOptimization::createPath()
    {
        initialize();

        std::random_device rd;
        std::default_random_engine engine(rd());
        std::uniform_real_distribution<double> rand01(0.0, 1.0);

        const double spread = 1.0;
        std::uniform_real_distribution<double> rand_spread(-spread, spread);

        std::vector<Eigen::VectorXd> positions(particle_num_, Eigen::VectorXd(2));
        std::vector<Eigen::VectorXd> velocities(particle_num_, Eigen::VectorXd::Zero(2));
        std::vector<Eigen::VectorXd> pbest(particle_num_, Eigen::VectorXd(2));
        std::vector<double> pbest_val(particle_num_, std::numeric_limits<double>::infinity());

        Eigen::VectorXd gbest(2);
        gbest[0] = robot_.position.x;
        gbest[1] = robot_.position.y;
        double gbest_val = std::numeric_limits<double>::infinity();

        for (int i = 0; i < particle_num_; ++i) {
            positions[i][0] = robot_.position.x + rand_spread(engine);
            positions[i][1] = robot_.position.y + rand_spread(engine);
            pbest[i] = positions[i];

            double val = objective_function(positions[i]);
            pbest_val[i] = val;
            if (val < gbest_val) {
                gbest_val = val;
                gbest     = positions[i];
            }
        }

        gbest_history_.reserve(max_iteration_ + 1);
        particles_history_.reserve(max_iteration_);
        gbest_history_.push_back(gbest);

        for (int iter = 0; iter < max_iteration_; ++iter) {
            particles_history_.emplace_back(particle_num_);
            auto& current_particles = particles_history_.back();

            for (int i = 0; i < particle_num_; ++i) {
                double r1 = rand01(engine);
                double r2 = rand01(engine);

                velocities[i] = weight_velocity_ * velocities[i]
                               + weight_pbest_ * r1 * (pbest[i] - positions[i])
                               + weight_gbest_ * r2 * (gbest    - positions[i]);

                positions[i] += velocities[i];
                current_particles[i] = positions[i];

                double val = objective_function(positions[i]);
                if (val < pbest_val[i]) {
                    pbest_val[i] = val;
                    pbest[i]     = positions[i];
                }
                if (val < gbest_val) {
                    gbest_val = val;
                    gbest     = positions[i];
                }
            }

            gbest_history_.push_back(gbest);
        }

        utility::vec_to_path(gbest_history_, path_);
    }

    void ParticleSwarmOptimization::createSurface(std::vector<Point>& points_output, double /*time*/)
    {
        points_output.clear();

        const double x_min = -5.0, x_max = 5.0;
        const double y_min = -5.0, y_max = 5.0;
        const double resolution = 0.1;

        for (double x = x_min; x <= x_max; x += resolution) {
            for (double y = y_min; y <= y_max; y += resolution) {
                points_output.push_back(Point(x, y, objective_function(x, y)));
            }
        }
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

    void ParticleSwarmOptimization::setCalculationParam(int pnum, int max_itr)
    {
        particle_num_  = pnum;
        max_iteration_ = max_itr;
    }

    void ParticleSwarmOptimization::setWeight(double wv, double wp, double wg)
    {
        weight_velocity_ = wv;
        weight_pbest_    = wp;
        weight_gbest_    = wg;
    }

    void ParticleSwarmOptimization::setThresholdDistance(double d)
    {
        threshold_distance_to_obstacle_ = d;
    }

    void ParticleSwarmOptimization::clearObstacles()
    {
        obstacles_.clear();
    }

    void ParticleSwarmOptimization::getPath(std::vector<Pose>& path)
    {
        path = path_;
    }

    void ParticleSwarmOptimization::getBestHistory(std::vector<Eigen::VectorXd>& path)
    {
        path = gbest_history_;
    }

    void ParticleSwarmOptimization::getParticlesHistory(std::vector<std::vector<Eigen::VectorXd>>& path)
    {
        path = particles_history_;
    }

} // namespace potbot_lib
