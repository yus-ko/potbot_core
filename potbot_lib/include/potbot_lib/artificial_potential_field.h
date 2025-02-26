#ifndef H_POTENTIALFIELD_
#define H_POTENTIALFIELD_

#include <potbot_lib/utility.h>
#include <potbot_lib/field.h>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

namespace potbot_lib{

    class ArtificialPotentialField : public potential::Field{
        protected:
        
            double weight_attraction_field_             = 0.1;
            double weight_repulsion_field_              = 0.1;
            double distance_threshold_repulsion_field_  = 0.3;  //単位:メートル

            Point robot_;
            Point goal_;
            std::vector<Point> obstacles_;

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
    };
}

#endif	// H_POTENTIALFIELD_