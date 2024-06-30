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
    }

}