#ifndef _H_POTENTIALFIELD_
#define _H_POTENTIALFIELD_

#include <ros/ros.h>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>

namespace potbot_lib{

    namespace Potential{

        enum GridInfo {
            IS_OBSTACLE,
            IS_GOAL,
            IS_ROBOT,
            IS_REPULSION_FIELD_INSIDE,
            IS_REPULSION_FIELD_EDGE,
            IS_PLANNED_PATH,
            IS_AROUND_GOAL,
            IS_LOCAL_MINIMUM
        };

        typedef struct {
                size_t index                = 0;
                double x                    = 0;
                double y                    = 0;
                double value                = 0;
                double attraction           = 0;
                double repulsion            = 0;
                double potential            = 0;
                size_t row                  = 0;
                size_t col                  = 0;
                std::vector<bool> states    = {false, false, false, false, false, false, false, false};
            } FieldGrid;
        
        typedef struct {
                double width            = 0;                //単位:メートル
                double height           = 0;                //単位:メートル
                double resolution       = 1.0;              //単位:メートル
                size_t rows             = 3;
                size_t cols             = 3;
                double x_shift          = 0;             
                double y_shift          = 0;
                double x_min            = 0;
                double x_max            = 0; 
                double y_min            = 0; 
                double y_max            = 0; 
            } FieldHeader;

        class Field{
            protected:

                FieldHeader header_;
                std::vector<FieldGrid> values_;

            public:
                Field(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double origin_x = 0.0, double origin_y = 0.0);
                ~Field();

                void init_field(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double origin_x = 0.0, double origin_y = 0.0);

                void set_values(std::vector<FieldGrid>& values);
                void set_value(FieldGrid value);

                void set_field_info(size_t index = 0, size_t meta = Potential::GridInfo::IS_OBSTACLE, bool value = false);
                void search_field_info(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode = "and");
                void search_field_info(std::vector<size_t>& result, const size_t term);

                int check_index(auto index);

                FieldHeader get_header();
                std::vector<FieldGrid>* get_values();
                FieldGrid get_value(size_t index = 0);
                FieldGrid get_value(double x = 0, double y = 0);

                size_t get_field_index(double x = 0, double y = 0);
                size_t get_field_index(size_t row = 0, size_t col = 0);
                std::vector<double> get_field_coordinate(size_t index = 0);

                void get_square_index(std::vector<size_t>& search_indexes, size_t centor_row = 0, size_t centor_col = 0, size_t range = 1);

                void to_pcl2(sensor_msgs::PointCloud2& pcl_msg);

                void info_filter(Field& field, const std::vector<size_t> terms, const std::string mode = "and");
                void info_filter(Field& field, const size_t term);
        };
    }

    class APF : public Potential::Field{
        protected:
        
            double weight_attraction_field_             = 0.1;
            double weight_repulsion_field_              = 0.1;
            double distance_threshold_repulsion_field_  = 0.3;  //単位:メートル

            std::vector<double> robot_                  = {0,0};
            std::vector<double> goal_                   = {0,0};
            std::vector<std::vector<double>> obstacles_;

            Potential::Field potential_field_;

        public:
            
            APF(size_t rows = 3, size_t cols = 3, double resolution = 1.0,
                double weight_attraction_field              = 0.1,
                double weight_repulsion_field               = 0.1,
                double distance_threshold_repulsion_field   = 0.3,
                double field_origin_x                       = 0,
                double field_origin_y                       = 0);
            ~APF();

            void init_potential_field(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double field_origin_x = 0.0, double field_origin_y = 0.0);

            void set_goal(size_t index = 0);
            void set_robot(size_t index = 0);
            void set_obstacle(size_t index = 0);

            void set_goal(double x = 0, double y = 0);
            void set_robot(double x = 0, double y = 0);
            void set_obstacle(double x = 0, double y = 0);

            void get_attraction_field(Potential::Field& field);
            void get_repulsion_field(Potential::Field& field);
            void get_potential_field(Potential::Field& field);

            std::vector<double> get_goal();
            std::vector<double> get_robot();
            std::vector<std::vector<double>> get_obstacles();

            void create_potential_field();
    };
}

#endif	// _H_POTENTIALFIELD_