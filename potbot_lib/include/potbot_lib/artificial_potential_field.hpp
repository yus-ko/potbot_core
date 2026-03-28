#ifndef HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_
#define HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_

#include <potbot_lib/utility.hpp>
#include <potbot_lib/field.hpp>
#include <eigen3/Eigen/Dense>

namespace potbot_lib{

    class ArtificialPotentialField : public potential::Field{
        protected:
        
            double weight_attraction_field_             = 0.1;
            double weight_repulsion_field_              = 0.1;
            double distance_threshold_repulsion_field_  = 0.3;  //単位:メートル
            double vortex_angle_                        = 0.0;  //渦巻き力の回転角度（デフォルト0で既存動作を保持）

            Point robot_;
            Point goal_;
            std::vector<Point> obstacles_;

            struct VirtualObstacle {
                double x, y;
                int lifetime;
            };
            std::vector<VirtualObstacle> virtual_obstacles_;

        public:
            
            ArtificialPotentialField(size_t rows = 3, size_t cols = 3, double resolution = 1.0,
                double weight_attraction_field              = 0.1,
                double weight_repulsion_field               = 0.1,
                double distance_threshold_repulsion_field   = 0.3,
                double field_origin_x                       = 0,
                double field_origin_y                       = 0);
            ~ArtificialPotentialField(){};

            void initPotentialField(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double field_origin_x = 0.0, double field_origin_y = 0.0);

            void setGoal(size_t index = 0);
            void setRobot(size_t index = 0);
            void setObstacle(size_t index = 0);
            void clearObstacles();

            void setParams(double wa, double wr, double dtr);
            void setVortexAngle(double angle);

            void addVirtualObstacle(double x, double y, int lifetime = 1);
            void clearVirtualObstacles();
            void decrementVirtualObstacleLifetimes();

            void setGoal(double x = 0, double y = 0);

            void setRobot(double x = 0, double y = 0);

            void setObstacle(double x = 0, double y = 0);
            
            void setObstacle(const Eigen::Vector2d& vec);

            void getAttractionField(potential::Field& field);
            void getRepulsionField(potential::Field& field);

            Point getGoal();
            Point getRobot();
            void getObstacles(std::vector<Point>& obs);

            double getDistanceThresholdRepulsionField(){return distance_threshold_repulsion_field_;};

            void createPotentialField();

            // グリッド非依存の解析的APF力ベクトル計算（リアルタイム制御用）
            void getForce(double rx, double ry,
                          double target_x, double target_y,
                          double& fx, double& fy) const;
    };
}

#endif // HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_