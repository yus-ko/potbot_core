#ifndef _H_PATHPLANNER_
#define _H_PATHPLANNER_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <potbot_lib/Utility.h>
#include <potbot_lib/PotentialField.h>

namespace potbot_lib{

    namespace PathPlanner{

        void get_path_msg_from_csv(nav_msgs::Path& path_msg,const std::string& csv_fullpath);

        class APFPathPlanner : public APF{
            private:
                double __nCr(double n, double r);
                std::vector<std::vector<Potential::FieldGrid>> loop_edges_;
                void __sort_repulsion_edges();
                void __get_repulsion_edges(std::vector<Potential::FieldGrid>& edges_clockwise, std::vector<Potential::FieldGrid>& edges_counterclockwise, size_t row_centor, size_t col_centor);
                size_t __get_smaller_potential_index(size_t centor_index, double potential_value);
            public:
                APFPathPlanner(size_t rows = 3, size_t cols = 3, double resolution = 1.0,
                double weight_attraction_field              = 0.1,
                double weight_repulsion_field               = 0.1,
                double distance_threshold_repulsion_field   = 0.3,
                double apf_origin_x                         = 0.0,
                double apf_origin_y                         = 0.0);
                ~APFPathPlanner();
                
                void create_path(std::vector<std::vector<double>> &path, double init_robot_pose = 0.0, double max_path_length = 6.0, size_t path_search_range = 1);
                void bezier(const std::vector<std::vector<double>> &path_control, std::vector<std::vector<double>> &path_interpolated);
                  
        };

    }
}

#endif	// _H_PATHPLANNER_