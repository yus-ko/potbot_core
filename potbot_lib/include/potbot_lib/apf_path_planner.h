#ifndef H_APF_PATH_PLANNER_
#define H_APF_PATH_PLANNER_

#include <random>
#include <potbot_lib/utility.h>
#include <potbot_lib/artificial_potential_field.h>

namespace potbot_lib{

    namespace path_planner{

        class APFPathPlanner{
            protected:
                ArtificialPotentialField *apf_;
                std::vector<std::vector<potential::FieldGrid>> loop_edges_;
                std::vector<Pose> path_;

                std::default_random_engine *random_engine_;
                std::uniform_real_distribution<double> *random_generator_double_;

                double max_path_length_ = 6.0;
                size_t path_search_range_ = 1;
                double path_weight_potential_ = 0.0;
                double path_weight_pose_ = 1.0;

            private:
                double combination(double n, double r);
                void sortRepulsionEdges();
                void getRepulsionEdges(std::vector<potential::FieldGrid>& edges_clockwise, std::vector<potential::FieldGrid>& edges_counterclockwise, size_t row_centor, size_t col_centor);
                size_t getSmallerPotentialIndex(size_t centor_index, double potential_value);

            public:
                APFPathPlanner(ArtificialPotentialField *apf);
                ~APFPathPlanner(){};

                void setParams(double maxp, size_t sr, double wpot, double wpos);

                bool createPathWithWeight(double init_robot_pose = 0.0);
                bool createPath(double init_robot_pose = 0.0);
                bool bezier();
                
                void getPath(std::vector<Pose>& path);
                  
        };

    }
}

#endif	// H_APF_PATH_PLANNER_