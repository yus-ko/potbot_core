#ifndef HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_
#define HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_

#include <potbot_lib/utility.hpp>
#include <potbot_lib/field.hpp>
#include <eigen3/Eigen/Dense>
#include <unordered_map>

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

            // 空間インデックス: 障害物を格子セルにハッシュして近傍検索を高速化
            double spatial_cell_size_ = 0.0;
            struct SpatialHash {
                size_t operator()(const std::pair<int,int>& p) const {
                    return std::hash<long long>()(((long long)p.first << 32) | (unsigned int)p.second);
                }
            };
            std::unordered_map<std::pair<int,int>, std::vector<size_t>, SpatialHash> obstacle_grid_;
            void buildObstacleSpatialIndex();

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
            void createPotentialFieldRepulsionOnly();

            // グリッド非依存の解析的APF力ベクトル計算（リアルタイム制御用）
            void getForce(double rx, double ry,
                          double target_x, double target_y,
                          double& fx, double& fy) const;

            // 差分更新: 障害物変化がなければ引力場のみ再計算（斥力場はキャッシュ）
            // 戻り値: true=差分更新した, false=全再計算が必要で実行した
            bool updatePotentialFieldIncremental(
                const std::vector<Point>& new_obstacles,
                double robot_move_threshold = 0.3,
                double obstacle_change_threshold = 0.05);

        private:
            // 前回の計算状態キャッシュ
            Point prev_robot_;
            Point prev_goal_;
            std::vector<Point> prev_obstacles_;
            bool has_cache_ = false;
            // 斥力場キャッシュ（グリッドセルごとの斥力値とIS_REPULSION_FIELD_INSIDEフラグ）
            std::vector<double> cached_repulsion_;
            std::vector<bool> cached_repulsion_inside_;
            size_t cached_grid_size_ = 0;

            // 引力場のみ再計算
            void updateAttractionField();
            // 局所解検出の再実行
            void updateLocalMinima();
    };
}

#endif // HPP_POTBOT_LIB_ARTIFICIAL_POTENTIAL_FIELD_