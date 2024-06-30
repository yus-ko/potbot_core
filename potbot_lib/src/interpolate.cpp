#include <potbot_lib/interpolate.h>

namespace potbot_lib{

    namespace interpolate{
        
        void linear(const std::vector<Eigen::Vector2d>& curve_in, int num_points, std::vector<Eigen::Vector2d>& curve_out)
        {
            size_t n = curve_in.size();
            if (n < 2 || num_points < 2)
            {
                curve_out = curve_in;
                return;
            }

            curve_out.clear();
            for (size_t i = 1; i < n; i++)
            {
                Eigen::Vector2d s = curve_in[i-1];
                Eigen::Vector2d e = curve_in[i];

                for (size_t j = 0; j <= num_points; j++) 
                {
                    double t = double(j) / double(num_points);
            	    curve_out.push_back((1 - t) * s + t * e);
                }
            }
        }

        void spline(const std::vector<Eigen::Vector2d>& curve_in, int num_points, std::vector<Eigen::Vector2d>& curve_out)
        {   
            int n = curve_in.size();
            if (n < 2 || num_points < 2)
            {
                curve_out = curve_in;
                return;
            }

            // Eigen::Matrixに変換
            Eigen::MatrixXd points(2, n);
            for (int i = 0; i < n; ++i) {
                points.col(i) = curve_in[i];
            }

            curve_out.clear();

            // スプライン補間のオブジェクトを作成
            typedef Eigen::Spline<double, 2> Spline2d;
            Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, std::min<int>(points.cols() - 1, 3));

            // 等間隔の点を生成
            for (int i = 0; i < num_points; ++i) {
                double t = static_cast<double>(i) / (num_points - 1);
                Eigen::Vector2d point = spline(t);
                if (point.allFinite()) curve_out.push_back(point);
            }
        }

        void bezier(const std::vector<Eigen::Vector2d>& curve_in, int num_points, std::vector<Eigen::Vector2d>& curve_out)
        {
            // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/

            int n = curve_in.size();
            if (n < 2 || num_points < 2)
            {
                curve_out = curve_in;
                return;
            }

            double x_min = curve_in.front()(0);
            double x_max = curve_in.front()(0);
            double y_min = curve_in.front()(1);
            double y_max = curve_in.front()(1);
            for(const auto& point : curve_in)
            {
                double x = point(0);
                double y = point(1);
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
            }

            curve_out.clear();
            int bezier_idx = 0;
            double inc = 1.0/double(num_points);
            // double interpolate_distance_threshold = sqrt(pow(path_control[0].position.x - path_control[1].position.x,2) + pow(path_control[0].position.y - path_control[1].position.y,2));
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = 0;
                double y = 0;
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    double a = utility::combination(n-1.0,i);
                    double b = pow(t,i);
                    double c = pow(1.0-t,n-i-1.0);
                    double x_inc = a * b * c * curve_in[size_t(i)](0);
                    double y_inc = utility::combination(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * curve_in[size_t(i)](1);
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
                // if (path_interpolated.size() > 1)
                // {
                //     double distance = sqrt(pow(x - path_interpolated.back().position.x,2) + pow(y - path_interpolated.back().position.y,2));
                //     if (distance > interpolate_distance_threshold) break;
                // }
                
                curve_out.push_back(Eigen::Vector2d(x,y));
            }
        }
    }
}
