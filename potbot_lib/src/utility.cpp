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
    }

}