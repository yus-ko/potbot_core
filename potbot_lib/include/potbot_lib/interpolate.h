#ifndef H_POTBOT_LIB_INTERPOLATE_
#define H_POTBOT_LIB_INTERPOLATE_

#include <potbot_lib/utility.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/Splines>

namespace potbot_lib{

    namespace interpolate{
        void linear(const std::vector<Eigen::Vector2d>& curve_in, int num_points, std::vector<Eigen::Vector2d>& curve_out);
        void spline(const std::vector<Eigen::Vector2d>& curve_in, int numPoints, std::vector<Eigen::Vector2d>& curve_out);
        void bezier(const std::vector<Eigen::Vector2d>& curve_in, int num_points, std::vector<Eigen::Vector2d>& curve_out);
    }
}

#endif	// H_POTBOT_LIB_INTERPOLATE_