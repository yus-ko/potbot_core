#include <potbot_lib/optimal_path_follower.hpp>

namespace potbot_lib
{
    namespace controller
    {

        void OptimalPathFollower::calculateCommand()
        {
            if (reachedTarget() || target_path_.empty())
                return;

            splitPath();

            if (optimization_method_ == "all_search")
            {
                createPlans();
                searchForBestPlan();
            }
            else if (optimization_method_ == "gradient")
            {
                searchForBestPlanWithGradient();
            }
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
        }

        void OptimalPathFollower::setTargetPath(const std::vector<potbot_lib::Pose> &path)
        {
            target_path_.clear();
            for (const auto &p : path)
            {
                target_path_.push_back(Eigen::Vector2d(p.position.x, p.position.y));
            }

            Pose g;
            if (!target_path_.empty())
            {
                g.position.x = target_path_.back()[0];
                g.position.y = target_path_.back()[1];
            }
        }

        void OptimalPathFollower::getPlans(std::vector<plan> &plans)
        {
            plans = plans_;
        }

        void OptimalPathFollower::getSplitPath(std::vector<Eigen::Vector2d> &path)
        {
            path = split_path_;
        }

        void OptimalPathFollower::getBestPath(std::vector<Eigen::Vector2d> &path)
        {
            path = best_plan_.path;
        }

        void OptimalPathFollower::getBestPlan(plan &plan)
        {
            plan = best_plan_;
        }

        void OptimalPathFollower::getBestCmd(double &v, double &omega)
        {
            v = best_plan_.linear_velocity;
            omega = best_plan_.angular_velocity;
        }

        void OptimalPathFollower::splitPath()
        {
            split_path_.clear();

            size_t closest_index = 0;
            double mind = 1e100;
            for (size_t i = 0; i < target_path_.size(); i++)
            {
                double d = (target_path_[i] - Eigen::Vector2d(x, y)).norm();
                if (d < mind)
                {
                    closest_index = i;
                    mind = d;
                }
            }

            double v_max = std::max(abs(linear_velocity_min_), abs(linear_velocity_max_));
            double limit_distance = v_max * time_end_;
            double inc_distance = v_max * time_increment_;
            size_t num_points = static_cast<size_t>(time_end_ / time_increment_);

            // パスの累積距離を計算
            std::vector<double> cumulative_dist;
            cumulative_dist.push_back(0.0);
            for (size_t i = closest_index + 1; i < target_path_.size(); i++)
            {
                double d = (target_path_[i] - target_path_[i - 1]).norm();
                double total = cumulative_dist.back() + d;
                cumulative_dist.push_back(total);
                if (total >= limit_distance)
                    break;
            }

            // inc_distance間隔で線形補間してsplit_pathを生成
            size_t seg = 0;
            for (size_t j = 0; j < num_points; j++)
            {
                double target_dist = inc_distance * (j + 1);
                if (target_dist > cumulative_dist.back())
                {
                    // パス末端を超えた場合、最後の点を繰り返す
                    split_path_.push_back(target_path_[closest_index + cumulative_dist.size() - 1]);
                    continue;
                }

                // target_distが含まれるセグメントを探す
                while (seg + 1 < cumulative_dist.size() && cumulative_dist[seg + 1] < target_dist)
                    seg++;

                if (seg + 1 < cumulative_dist.size())
                {
                    double seg_len = cumulative_dist[seg + 1] - cumulative_dist[seg];
                    double t = (seg_len > 1e-10) ? (target_dist - cumulative_dist[seg]) / seg_len : 0.0;
                    Eigen::Vector2d p = (1.0 - t) * target_path_[closest_index + seg]
                                        + t * target_path_[closest_index + seg + 1];
                    split_path_.push_back(p);
                }
                else
                {
                    split_path_.push_back(target_path_[closest_index + cumulative_dist.size() - 1]);
                }
            }
        }

        void OptimalPathFollower::searchForBestPlan()
        {
            double score_min = 1e100;
            double score_max = 1e-100;
            std::vector<double> score;
            for (const auto &plan : plans_)
            {
                size_t num_points = plan.path.size();
                if (num_points >= split_path_.size())
                    num_points = split_path_.size();

                double score_diff = 0;
                for (size_t i = 0; i < num_points; i++)
                {
                    score_diff += (plan.path[i] - split_path_[i]).norm();
                }

                score.push_back(score_diff);
            }

            auto min_it = std::min_element(score.begin(), score.end());
            int min_index = std::distance(score.begin(), min_it);

            best_plan_ = plans_[min_index];
            score_min = *min_it;

            auto max_value = *std::max_element(score.begin(), score.end());
        }

        void OptimalPathFollower::searchForBestPlanWithGradient()
        {
            plans_.clear();

            double x_init = x;
            double y_init = y;
            double th_init = yaw;
            double dt = time_increment_;
            double tau = time_end_;
            double v_min = linear_velocity_min_;
            double v_max = linear_velocity_max_;
            double omega_min = angular_velocity_min_;
            double omega_max = angular_velocity_max_;

            double v = this->v;
            double omega = this->omega;

            double score_min = 1e100;
            double score_min_threshold = 1e-2;
            double learning_rate = learning_rate_;
            size_t iteration_max = iteration_max_;

            size_t i = 0;
            for (i = 0; i < iteration_max; i++)
            {
                plan p;
                p.linear_velocity = v;
                p.angular_velocity = omega;
                p.delta_time = dt;
                p.end_time = tau;
                size_t num_points = split_path_.size();
                Eigen::Vector2d gradient(0, 0);
                // for (double t = 0; t < tau; t += dt)
                for (int j = 0; j < num_points; j++)
                {
                    double t = dt * j;
                    double th = th_init + omega * t;
                    double x = x_init + v * t * cos(th);
                    double y = y_init + v * t * sin(th);
                    p.path.push_back(Eigen::Vector2d(x, y));

                    double x2 = split_path_[j](0);
                    double y2 = split_path_[j](1);

                    double s1 = x - x2;
                    double s2 = y - y2;
                    double s3 = sqrt(pow(s1, 2) + pow(s2, 2));

                    double gv = t * (cos(th) * s1 + sin(th) * s2) / (s3);
                    // double gv =
                    //     2*t*sin(th)*(t*sin(th)*v-y2+y_init) + 2*t*cos(th)*(t*cos(th)*v-x2+x_init)/
                    //     2*sqrt(pow(t*sin(th)*v-y2+y_init,2)+pow(t*cos(th)-x2+x_init,2));

                    double gomega = -t * t * v * (sin(th) * s1 - cos(th) * s2) / (s3);
                    // double gomega =
                    //     t*t*v*((x2-x_init)*sin(th)+(y_init-y2)*cos(th))/
                    //     sqrt(pow(t*v*sin(th)-y2+y_init,2)+pow(t*v*cos(th)-x2+x_init,2));

                    gradient += Eigen::Vector2d(gv, gomega);
                }

                double sum = 0;
                for (size_t j = 0; j < num_points; j++)
                {
                    sum += (p.path[j] - split_path_[j]).norm();
                }

                double score = sum;

                plans_.push_back(p);

                if (score < score_min)
                {
                    best_plan_ = p;
                    score_min = score;
                }

                double v_inc = learning_rate * gradient(0);
                double omega_inc = learning_rate * gradient(1);
                // if (abs(v_inc) < score_min_threshold && abs(omega_inc) < score_min_threshold)
                // {
                //     i++;
                //     break;
                // }

                v -= v_inc;
                omega -= omega_inc;

                v = std::min(v, v_max);
                v = std::max(v, v_min);
                omega = std::min(omega, omega_max);
                omega = std::max(omega, omega_min);
            }
        }

        void OptimalPathFollower::createPlans()
        {
            plans_.clear();
            double x_init = x;
            double y_init = y;
            double th_init = yaw;
            double dt = time_increment_;
            double tau = time_end_;
            double v_min = linear_velocity_min_;
            double v_max = linear_velocity_max_;
            double v_delta = linear_velocity_increment_;
            double omega_min = angular_velocity_min_;
            double omega_max = angular_velocity_max_;
            double omega_delta = angular_velocity_increment_;

            for (double v = v_min; v <= v_max; v += v_delta)
            {
                for (double omega = omega_min; omega <= omega_max; omega += omega_delta)
                {
                    plan p;
                    p.linear_velocity = v;
                    p.angular_velocity = omega;
                    p.delta_time = dt;
                    p.end_time = tau;
                    double x = x_init, y = y_init, th = th_init;
                    for (double t = 0; t < tau; t += dt)
                    {
                        th += omega * dt;
                        x += v * dt * cos(th);
                        y += v * dt * sin(th);
                        p.path.push_back(Eigen::Vector2d(x, y));
                    }
                    plans_.push_back(p);
                }
            }
        }

        bool OptimalPathFollower::reachedTarget()
        {
            if (target_path_.empty())
                return true;
            Pose p;
            p.position.x = target_path_.back()[0];
            p.position.y = target_path_.back()[1];
            return getDistance(p) <= stop_margin_distance_;
        }
    }
}