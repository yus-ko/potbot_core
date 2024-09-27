#ifndef H_POTBOT_LIB_UTILITY_
#define H_POTBOT_LIB_UTILITY_

#include <vector>
#include <eigen3/Eigen/Dense>

namespace potbot_lib{

    const int SUCCESS = 1;
    const int FAIL = 0;

    typedef struct {
        int index=0;
        double x=0;
        double y=0;
        double r=0;
        double theta=0;
    } ScanPoint;

    struct Point{
        double x = 0;
        double y = 0;
        double z = 0;

        double norm() const {
            return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        }

        bool operator==(const Point& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Point& other) const {
            return !(*this == other);
        }
    };

    struct Pose{
        Point position;
        Point rotation;

        Eigen::Affine3d to_affine() const {
            Eigen::Affine3d aff = Eigen::Affine3d::Identity();
            aff.translation() << position.x, position.y, position.z;
            aff.rotate( Eigen::AngleAxisd(rotation.x, Eigen::Vector3d::UnitX())*
                        Eigen::AngleAxisd(rotation.y, Eigen::Vector3d::UnitY())*
                        Eigen::AngleAxisd(rotation.z, Eigen::Vector3d::UnitZ()));
            return aff;
        }

        bool operator==(const Pose& other) const {
            return position == other.position && rotation == other.rotation;
        }

        bool operator!=(const Pose& other) const {
            return !(*this == other);
        }
    };

    namespace utility{
        
        void find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest);

        int get_index(const std::vector<Eigen::Vector2d>& vec, const Eigen::Vector2d& value);

        Eigen::Matrix2d get_rotate_matrix(double th);
        
        double combination(double n, double r);

        template <typename T>
        bool is_containing(const T& element, const std::vector<T>& vec)
        {
            return std::find(vec.begin(), vec.end(), element) != vec.end();
        };
    }
}

#endif	// H_POTBOT_LIB_UTILITY_