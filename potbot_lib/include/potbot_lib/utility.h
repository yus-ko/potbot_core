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
        double x,y,z;

        Point(double x_val = 0, double y_val = 0, double z_val = 0) : x(x_val), y(y_val), z(z_val){}
        Point(Eigen::Vector3d vec) : Point(vec.x(), vec.y(), vec.z()){}
        Point(Eigen::Matrix3d rotmat) : Point((Eigen::Vector3d)rotmat.eulerAngles(0, 1, 2)){}

        Eigen::Vector3d to_translation() const {
            return Eigen::Vector3d{x,y,z};
        }

        Eigen::Matrix3d to_rotation() const {
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = 
                    Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
            return rotation_matrix;
        }

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
        
        Pose(double x_val = 0, double y_val = 0, double z_val = 0,
            double roll = 0, double pitch = 0, double yaw = 0) : 
            position(x_val, y_val, z_val), rotation(roll, pitch, yaw){}
        Pose(Eigen::Affine3d aff) : position((Eigen::Vector3d)aff.translation()), rotation((Eigen::Matrix3d)aff.rotation()) {}

        Eigen::Affine3d to_affine() const {
            Eigen::Affine3d aff = Eigen::Affine3d::Identity();
            aff.translation() = position.to_translation();
            // aff.linear() = rotation.to_rotation();
            aff.rotate(rotation.to_rotation());
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