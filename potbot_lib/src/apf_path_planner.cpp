#include <potbot_lib/apf_path_planner.h>

namespace potbot_lib{

    namespace path_planner{

        // void get_search_index_APF(Field& field, std::vector<size_t>, )
        APFPathPlanner::APFPathPlanner(ArtificialPotentialField *apf) : apf_(NULL)
        {
            apf_ = apf;
        }

        void APFPathPlanner::setParams(double maxp, size_t sr, double wpot, double wpos)
        {
            max_path_length_ = maxp;
            path_search_range_ = sr;
            path_weight_potential_ = wpot;
            path_weight_pose_ = wpos;
        }

        bool APFPathPlanner::createPathWithWeight(double init_robot_pose)
        {
            path_.clear();
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre, J_min_tmp;
            double P_min;
            double path_length  = 0;
            size_t range        = path_search_range_;
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();
            double scale = std::pow(10, 3);

            for (auto value : (*field_values))
            {
                if (value.states[potential::GridInfo::IS_ROBOT])
                {
                    pf_idx_min  = value.index;
                    center_x    = value.x;
                    center_y    = value.y;
                    J_min_pre   = value.value;
                    P_min       = value.value;
                    center_row  = value.row;
                    center_col  = value.col;
                    break;
                }
            }
            path_.push_back(Pose{center_x, center_y});
            (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            sortRepulsionEdges();

            double theta_pre = init_robot_pose;
            double x_pre     = path_.back().position.x;
            double y_pre     = path_.back().position.y;

            bool solving_local_minimum = false;
            bool change_weight = false;
            // J_min_pre = 1e10;
            while ((*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL] == false && 
                    path_length <= max_path_length_)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path_.size() > 100) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                apf_->getSquareIndex(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        if (PotentialValue < P_min) 
                        {
                            solving_local_minimum       = false;
                            P_min                       = PotentialValue;
                            pf_idx_min                  = idx;
                        }
                    }
                    // if (solving_local_minimum == true)
                    // {
                    //     change_weight = true;
                    //     //j_minの保存
                    //     //姿勢のround
                    //     J_min_tmp = J_min;
                    //     continue;
                    // }
                    // else
                    // {
                    //     // J_min = J_min_tmp;
                    //     change_weight = false;
                    // }
                    
                }
                else
                {
                        
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        double x                = (*field_values)[idx].x;
                        double y                = (*field_values)[idx].y;
                        
                        double theta            = atan2(y-y_pre,x-x_pre);
                        double posediff         = abs(theta - theta_pre);
                                posediff         = std::floor(posediff * scale) / scale;

                        double wu               = path_weight_potential_;
                        double w_theta          = path_weight_pose_;
                        double J                = wu*PotentialValue + w_theta*posediff;

                        if (J <= J_min) 
                        {
                            solving_local_minimum       = false;
                            J_min                       = J;
                            pf_idx_min                  = idx;
                        }
                    }
                    
                }
                
                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path_.back().position.x,2) + pow(py - path_.back().position.y,2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;
                P_min = (*field_values)[pf_idx_min].value;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path_.back().position.y,px-path_.back().position.x);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                Pose p{px,py};
                if (p == path_.end()[-1] && (p == path_.end()[-2])) break;

                path_.push_back(Pose{px, py});
                (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                
            }
            return true;
        }

        bool APFPathPlanner::createPath(double init_robot_pose)
        {
            path_.clear();
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre;
            double path_length  = 0;
            size_t range        = path_search_range_;
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();

            for (auto value : (*field_values))
            {
                if (value.states[potential::GridInfo::IS_ROBOT])
                {
                    pf_idx_min  = value.index;
                    center_x    = value.x;
                    center_y    = value.y;
                    J_min_pre   = value.value;
                    center_row  = value.row;
                    center_col  = value.col;
                    break;
                }
            }
            path_.push_back(Pose{center_x, center_y});
            (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            sortRepulsionEdges();

            double theta_pre = init_robot_pose;
            double x_pre     = path_.back().position.x;
            double y_pre     = path_.back().position.y;

            bool solving_local_minimum = false;
            bool change_weight = false;
            while ((*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL] == false && 
                    path_length <= max_path_length_)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path_.size() > 100) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                apf_->getSquareIndex(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        double x                = (*field_values)[idx].x;
                        double y                = (*field_values)[idx].y;
                        
                        double theta = atan2(y-y_pre,x-x_pre);
                        double posediff         = abs(theta - theta_pre);

                        double wu               = 1.0;
                        double w_theta          = 0;
                        if (change_weight == true)
                        {
                            wu                  = 0.0;
                            w_theta             = 1.0;
                        }
                        double J                = wu*PotentialValue + w_theta*posediff;

                        if (J < J_min) 
                        {
                            solving_local_minimum       = false;
                            J_min                       = J;
                            pf_idx_min                  = idx;
                        }
                    }
                    
                }
                else
                {

                    std::vector<potential::FieldGrid> edges_clockwise, edges_counterclockwise;
                    getRepulsionEdges(edges_clockwise, edges_counterclockwise, center_row, center_col);




                    // path.clear();
                    // for (size_t i = 0; i < edges_clockwise.size(); i++)
                    // {
                    //     path.push_back({edges_clockwise[i].x, edges_clockwise[i].y});
                    //     (*field_values)[edges_clockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;

                    // path.clear();
                    // for (size_t i = 0; i < edges_counterclockwise.size(); i++)
                    // {
                    //     path.push_back({edges_counterclockwise[i].x, edges_counterclockwise[i].y});
                    //     (*field_values)[edges_counterclockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;




                    // double distance_to_goal_clockwise = std::numeric_limits<double>::infinity();
                    size_t path_length_clockwise;
                    for (path_length_clockwise = 0;path_length_clockwise < edges_clockwise.size()/2; path_length_clockwise++)
                    {
                        if (getSmallerPotentialIndex(edges_clockwise[path_length_clockwise].index, J_min) != edges_clockwise[path_length_clockwise].index)
                        {
                            break;
                        }
                    }

                    size_t path_length_counterclockwise;
                    for (path_length_counterclockwise = 0;path_length_counterclockwise < edges_counterclockwise.size()/2; path_length_counterclockwise++)
                    {
                        if (getSmallerPotentialIndex(edges_counterclockwise[path_length_counterclockwise].index, J_min) != edges_counterclockwise[path_length_counterclockwise].index)
                        {
                            break;
                        }
                    }
                    
                    if(path_length_clockwise < path_length_counterclockwise)
                    {
                        for (size_t i = 0; i < path_length_clockwise; i++)
                        {
                            path_.push_back(Pose{edges_clockwise[i].x, edges_clockwise[i].y});
                            (*field_values)[edges_clockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                        }
                        pf_idx_min = edges_clockwise[path_length_clockwise].index;
                    }
                    else
                    {
                        for (size_t i = 0; i < path_length_counterclockwise; i++)
                        {
                            path_.push_back(Pose{edges_counterclockwise[i].x, edges_counterclockwise[i].y});
                            (*field_values)[edges_counterclockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                        }
                        pf_idx_min = edges_counterclockwise[path_length_counterclockwise].index;
                    }

                    solving_local_minimum   = false;
                }
                
                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path_.back().position.x,2) + pow(py - path_.back().position.y,2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path_.back().position.y,px-path_.back().position.x);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                Pose p{px,py};
                if ((p == path_.end()[-1] && p == path_.end()[-2])) break;

                path_.push_back(Pose{px, py});
                (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                
            }
            return true;
        }

        size_t APFPathPlanner::getSmallerPotentialIndex(size_t centor_index, double potential_value)
        {
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();

            size_t centor_row = (*field_values)[centor_index].row;
            size_t centor_col = (*field_values)[centor_index].col;

            if(potential_value < 0)
            {
                potential_value = 0;
            }
            
            std::vector<size_t> search_indexes;
            apf_->getSquareIndex(search_indexes, centor_row, centor_col, 1);
            if (search_indexes.empty()) return centor_index;

            double distance_from_centor_to_goal = sqrt(pow((*field_values)[centor_index].x - apf_->getGoal().x,2)+pow((*field_values)[centor_index].y - apf_->getGoal().y,2));

            for (auto idx : search_indexes)
            {
                if ((*field_values)[idx].value < potential_value) 
                {
                    double distance_from_idx_to_goal = sqrt(pow((*field_values)[idx].x - apf_->getGoal().x,2)+pow((*field_values)[idx].y - apf_->getGoal().y,2));
                    if (distance_from_idx_to_goal < distance_from_centor_to_goal)
                    {
                        return idx;
                    }
                }
            }

            return centor_index;

        }

        void APFPathPlanner::sortRepulsionEdges()
        {
            loop_edges_.clear();
            
            potential::Field edge_field;
            apf_->infoFilter(edge_field, potential::GridInfo::IS_REPULSION_FIELD_EDGE);
            std::vector<potential::FieldGrid>* edge_values = edge_field.getValues();
            if((*edge_values).empty()) return;

            float next_grid_distance = sqrt(2.0) * apf_->getHeader().resolution;
            // next_grid_distance+=0.2;
            float scale = std::pow(10, 3);
            next_grid_distance =  std::ceil(next_grid_distance * scale) / scale;

            std::vector<potential::FieldGrid> sorted_values;
            sorted_values.push_back((*edge_values)[0]);
            (*edge_values).erase((*edge_values).begin());
            while (!(*edge_values).empty())
            {
                float x = sorted_values.back().x;
                float y = sorted_values.back().y;
                float min_distance = std::numeric_limits<float>::infinity();
                size_t min_index;
                for (size_t index = 0; index < (*edge_values).size(); index++)
                {
                    float distance = sqrt(pow(x - (*edge_values)[index].x, 2) + pow(y - (*edge_values)[index].y, 2));
                    if (distance <  min_distance)
                    {
                        min_distance = distance;
                        min_index = index;
                    }
                }

                if (min_distance > next_grid_distance)
                {
                    loop_edges_.push_back(sorted_values);
                    sorted_values.clear();
                }
                sorted_values.push_back((*edge_values)[min_index]);
                (*edge_values).erase((*edge_values).begin() + min_index);
            }
            loop_edges_.push_back(sorted_values);

            // std::vector<std::vector<potbot_lib::potential::FieldGrid>> merged;
            // for (const auto& points : loop_edges_) 
            // {
            //     if (merged.empty()) 
            //     {
            //         merged.push_back(points);
            //     } 
            //     else 
            //     {
            //         float s_x = merged.back().back().x;
            //         float s_y = merged.back().back().y;
            //         float e_x = merged.back().front().x;
            //         float e_y = merged.back().front().y;

            //         float distance1 = sqrt(pow(s_x-points.front().x,2)+pow(s_y-points.front().y,2));
            //         float distance2 = sqrt(pow(e_x-points.front().x,2)+pow(e_y-points.front().y,2));
            //         float distance3 = sqrt(pow(s_x-points.back().x,2)+pow(s_y-points.back().y,2));
            //         float distance4 = sqrt(pow(e_x-points.back().x,2)+pow(e_y-points.back().y,2));

            //         if (distance1 <= next_grid_distance || distance2 <= next_grid_distance || distance3 <= next_grid_distance || distance4 <= next_grid_distance) 
            //         {
            //             for (const auto& p:points)
            //             {
            //                 merged.back().push_back(p);
            //             }
            //         }
            //         else
            //         { 
            //             merged.push_back(points);
            //         }
            //     }
            // }
            // loop_edges_ = merged;

        }

        void APFPathPlanner::getRepulsionEdges(std::vector<potential::FieldGrid>& edges_clockwise, std::vector<potential::FieldGrid>& edges_counterclockwise, size_t row_centor, size_t col_centor)
        {
            // auto loops = loop_edges_[0];
            // edges_clockwise.push_back(loops[0]);
            // for (size_t i = 1; i < loops.size(); i++)
            // {
            //     double xi = loops[i].x;
            //     double yi = loops[i].y;
            //     double xi_1 = loops[i-1].x;
            //     double yi_1 = loops[i-1].y;
            //     double distance = sqrt(pow(xi-xi_1,2)+pow(yi-yi_1,2));
            //     double ref = sqrt(0.11*0.11+0.11*0.11);
            //     if (distance <= ref)
            //     {
            //         edges_clockwise.push_back(loops[i]);
            //     }
            //     else
            //     {
            //         break;
            //     }
                
            // }
            // return;
            

            size_t start_index = apf_->getFieldIndex(row_centor, col_centor);

            float min_distance = std::numeric_limits<float>::infinity();
            float start_x = apf_->getValue(start_index).x;
            float start_y = apf_->getValue(start_index).y;
            // std::vector<potential::FieldGrid> near_edges;
            for (const auto& loops : loop_edges_)
            {
                for (const auto& value : loops)
                {
                    float distance = sqrt(pow(start_x - value.x,2) + pow(start_y - value.y,2));
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        start_index = value.index;
                        // near_edges = loops;
                    }
                }
            }

            for (const auto& loops : loop_edges_)
            {
                size_t shift_num = 0;
                for (const auto& value : loops)
                {
                    if(value.index == start_index)
                    {   
                        // edges_counterclockwise.push_back(loops[0]);
                        // for (size_t i = 1; i < loops.size(); i++)
                        // {
                        //     double xi = loops[i].x;
                        //     double yi = loops[i].y;
                        //     double xi_1 = loops[i-1].x;
                        //     double yi_1 = loops[i-1].y;
                        //     double distance = sqrt(pow(xi-xi_1,2)+pow(yi-yi_1,2));
                        //     double ref = sqrt(0.11*0.11+0.11*0.11);
                        //     if (distance <= ref)
                        //     {
                        //         edges_counterclockwise.push_back(loops[i]);
                        //     }
                        //     else
                        //     {
                        //         break;
                        //     }
                            
                        // }
                        

                        edges_counterclockwise  = loops;
                        std::rotate(edges_counterclockwise.begin(), edges_counterclockwise.begin() + shift_num, edges_counterclockwise.end());
                        edges_clockwise         = edges_counterclockwise;
                        std::reverse(edges_clockwise.begin(), edges_clockwise.end());
                        std::rotate(edges_clockwise.rbegin(), edges_clockwise.rbegin() + 1, edges_clockwise.rend());
                        return;
                    }
                    shift_num++;
                }
            }
        }

        bool APFPathPlanner::bezier()
        {
            // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/
            
            const std::vector<Pose> &path_control = path_;
            std::vector<Pose> path_interpolated;

            if (path_control.size() < 2) return false;

            double x_min = path_control[0].position.x;
            double x_max = path_control[0].position.x;
            double y_min = path_control[0].position.y;
            double y_max = path_control[0].position.y;
            for(const auto& point : path_control)
            {
                double x = point.position.x;
                double y = point.position.y;
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
            }

            double n = path_control.size();

            int bezier_idx = 0;
            double inc = 1.0/double(n*10);
            double interpolate_distance_threshold = sqrt(pow(path_control[0].position.x - path_control[1].position.x,2) + pow(path_control[0].position.y - path_control[1].position.y,2));
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = 0;
                double y = 0;
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    double a = combination(n-1.0,i);
                    double b = pow(t,i);
                    double c = pow(1.0-t,n-i-1.0);
                    double x_inc = a * b * c * path_control[size_t(i)].position.x;
                    double y_inc = combination(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)].position.y;
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
                if (path_interpolated.size() > 1)
                {
                    double distance = sqrt(pow(x - path_interpolated.back().position.x,2) + pow(y - path_interpolated.back().position.y,2));
                    if (distance > interpolate_distance_threshold) break;
                }
                
                path_interpolated.push_back({x,y});
            }
            
            path_ = path_interpolated;
            return true;
        }

        void APFPathPlanner::getPath(std::vector<Pose>& path)
        {
            path = path_;
        }

        double APFPathPlanner::combination(double n, double r)
        {
            double top = 1.0;
            double bottom = 1.0;

            for(double i = 0.0; i < r; i++)
            {
                top *= n-i;
            }

            for(double i = 0.0; i < r; i++)
            {
                bottom *= i+1.0;
            }
            
            double ans = top/bottom;
            if (std::isnan(ans))
            {
                ans = 0;
            }
            if (std::isinf(ans))
            {
                ans = 1e100;
            }
            return ans;
        }
    }
}