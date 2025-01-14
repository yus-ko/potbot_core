#include <potbot_lib/utility.h>

namespace potbot_lib{

    namespace utility{

        void find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest)
        {
            double minDistance = std::numeric_limits<double>::max();

            for (const auto& vec : vectors) {
                double distance = (vec - target).norm();
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = vec;
                }
            }
        }

        int get_index(const std::vector<Eigen::Vector2d>& vec, const Eigen::Vector2d& value)
        {
            auto it = std::find(vec.begin(), vec.end(), value);
            if (it != vec.end()) {
                return std::distance(vec.begin(), it);
            } else {
                return -1; // Return -1 if the element is not found
            }
        }

        Eigen::Matrix2d get_rotate_matrix(double th)
        {
            Eigen::Matrix2d R(2,2);
            R << cos(th), -sin(th),
                sin(th), cos(th);
            return R;
        }

        double combination(double n, double r)
        {
            double top = 1.0;
            double bottom = 1.0;

            for(double i = 0.0; i < r; i++)
            {
                top *= n-i;
            }

            for(double i = 0.0; i < r; i++)
            {
                bottom *= i+1.0;
            }
            
            double ans = top/bottom;
            if (std::isnan(ans))
            {
                ans = 0;
            }
            if (std::isinf(ans))
            {
                ans = 1e100;
            }
            return ans;
        }

        void vec_to_path(const std::vector<Eigen::VectorXd>& vectors, std::vector<Pose>& path)
        {
            path.clear();
			for (const auto& vec : vectors)
			{
				path.push_back(Pose(vec(0), vec(1)));
			}
        }

        bool bezier(const std::vector<Pose> path_raw, std::vector<Pose>& path_interpolated)
        {   
            path_interpolated.clear();
            const std::vector<Pose> &path_control = path_raw;

            if (path_control.size() < 2 || path_control.size() > 100)
            {
                path_interpolated = path_raw;
                return false;
            } 

            double x_min = path_control[0].position.x;
            double x_max = path_control[0].position.x;
            double y_min = path_control[0].position.y;
            double y_max = path_control[0].position.y;
            for(const auto& point : path_control)
            {
                double x = point.position.x;
                double y = point.position.y;
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
            }

            double n = path_control.size();

            int bezier_idx = 0;
            double inc = 1.0/double(n*10);
            double interpolate_distance_threshold = sqrt(pow(path_control[0].position.x - path_control[1].position.x,2) + pow(path_control[0].position.y - path_control[1].position.y,2));
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = 0;
                double y = 0;
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    double a = combination(n-1.0,i);
                    double b = pow(t,i);
                    double c = pow(1.0-t,n-i-1.0);
                    double x_inc = a * b * c * path_control[size_t(i)].position.x;
                    double y_inc = combination(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)].position.y;
                    if (std::isnan(x_inc) || std::isinf(x_inc))
                    {
                        x_inc = 0;
                    }
                    if (std::isnan(y_inc) || std::isinf(y_inc))
                    {
                        y_inc = 0;
                    }
                    
                    x += x_inc;
                    y += y_inc;
                }
                // if (x > x_max || x < x_min || y > y_max || y < y_min)
                // {
                //     path_interpolated = path_control;
                //     return;
                // }
                if (path_interpolated.size() > 1)
                {
                    double distance = sqrt(pow(x - path_interpolated.back().position.x,2) + pow(y - path_interpolated.back().position.y,2));
                    if (distance > interpolate_distance_threshold) break;
                }
                
                path_interpolated.push_back({x,y});
            }
            return true;
        }
    }

}