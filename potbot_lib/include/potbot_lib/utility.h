#ifndef H_POTBOT_LIB_UTILITY_
#define H_POTBOT_LIB_UTILITY_

#include <vector>
#include <eigen3/Eigen/Dense>

namespace potbot_lib{

    const int SUCCESS = 1;
    const int FAIL = 0;

    // const int MEGAROVER = 0;
    // const int TURTLEBOT3 = 1;
    // const int BEEGO = 2;

    const int DEAD_RECKONING = 0;
    const int PARTICLE_FILTER = 1;

    const int CSV_PATH = 0;
    const int POTENTIAL_METHOD = 1;

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
    }
}

#endif	// H_POTBOT_LIB_UTILITY_