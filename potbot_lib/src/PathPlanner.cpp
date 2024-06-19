#include <potbot_lib/PathPlanner.h>

namespace potbot_lib{

    namespace PathPlanner{

        void get_path_msg_from_csv(nav_msgs::Path& path_msg,const std::string& csv_fullpath)
        {
            nav_msgs::Path init;
            path_msg = init;

            std::string str_buf;
            std::string str_conma_buf;
            std::ifstream ifs_csv_file(csv_fullpath);

            double line_buf[3];
            while (getline(ifs_csv_file, str_buf)) 
            {    
                
                std::istringstream i_stream(str_buf);// 「,」区切りごとにデータを読み込むためにistringstream型にする
                
                int i = 0;
                while (getline(i_stream, str_conma_buf, ',')) // 「,」区切りごとにデータを読み込む
                {
                    line_buf[i++] = std::stod(str_conma_buf);
                }
                geometry_msgs::PoseStamped pose;
                pose.pose = potbot_lib::utility::get_Pose(line_buf[0], line_buf[1], 0, 0, 0, line_buf[2]);
                path_msg.poses.push_back(pose);
            }
            
        }

        // void get_search_index_APF(Field& field, std::vector<size_t>, )
        APFPathPlanner::APFPathPlanner(size_t rows, size_t cols, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field, double apf_origin_x, double apf_origin_y) : 
        APF::APF(rows, cols, resolution,weight_attraction_field, weight_repulsion_field, distance_threshold_repulsion_field, apf_origin_x, apf_origin_y){}

        APFPathPlanner::APFPathPlanner(costmap_2d::Costmap2D* costmap, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field):
        APF::APF(costmap, weight_attraction_field, weight_repulsion_field, distance_threshold_repulsion_field){}

        void APFPathPlanner::get_loop_edges(visualization_msgs::MarkerArray& msg)
        {
            msg.markers.clear();
            int id = 0;
            for (const auto& loops:loop_edges_)
            {
                visualization_msgs::Marker marker;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.id = id++;
                marker.color = potbot_lib::color::get_msg(marker.id);
                marker.scale.x = 0.05;
                marker.lifetime = ros::Duration(0.5);
                marker.pose = utility::get_Pose();
                marker.ns = std::to_string(marker.id);
                for (const auto& point:loops)
                {
                    geometry_msgs::Point point_msg = utility::get_Point(point.x, point.y);
                    marker.points.push_back(point_msg);
                }
                msg.markers.push_back(marker);
            }
        }

        void APFPathPlanner::path_to_msg(std::vector<std::vector<double>> &path, std::vector<geometry_msgs::PoseStamped> &msg)
        {
            msg.clear();
            for (auto point : path)
            {
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.pose.position.x = point[0];
                pose_msg.pose.position.y = point[1];
                pose_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
                msg.push_back(pose_msg);
            }
        }

        void APFPathPlanner::path_to_msg(std::vector<std::vector<double>> &path, nav_msgs::Path &msg)
        {
            path_to_msg(path, msg.poses);
        }

        void APFPathPlanner::create_path_with_weight(std::vector<std::vector<double>> &path, double init_robot_pose, double max_path_length, size_t path_search_range, double path_weight_potential, double path_weight_pose)
        {
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre, J_min_tmp;
            double P_min;
            double path_length  = 0;
            size_t range        = path_search_range;
            std::vector<Potential::FieldGrid>* field_values;
            Field& field = potential_field_;
            field_values = field.get_values();
            double scale = std::pow(10, 3);

            for (auto value : (*field_values))
            {
                if (value.states[Potential::GridInfo::IS_ROBOT])
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
            path.push_back({center_x, center_y});
            (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;

            __sort_repulsion_edges();

            double theta_pre = init_robot_pose;
            double x_pre     = path.back()[0];
            double y_pre     = path.back()[1];

            bool solving_local_minimum = false;
            bool change_weight = false;
            // J_min_pre = 1e10;
            while ((*field_values)[pf_idx_min].states[Potential::GridInfo::IS_AROUND_GOAL] == false && 
                    path_length <= max_path_length)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path.size() > 100) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                field.get_square_index(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[Potential::GridInfo::IS_PLANNED_PATH] == true) continue;

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
                        if ((*field_values)[idx].states[Potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        double x                = (*field_values)[idx].x;
                        double y                = (*field_values)[idx].y;
                        
                        double theta            = atan2(y-y_pre,x-x_pre);
                        double posediff         = abs(theta - theta_pre);
                                posediff         = std::floor(posediff * scale) / scale;

                        double wu               = path_weight_potential;
                        double w_theta          = path_weight_pose;
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
                path_length += sqrt(pow(px - path.back()[0],2) + pow(py - path.back()[1],2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;
                P_min = (*field_values)[pf_idx_min].value;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path.back()[1],px-path.back()[0]);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                if ((std::vector<double>){px, py} == path.end()[-1] && (std::vector<double>){px, py} == path.end()[-2]) break;

                path.push_back({px, py});
                (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                
            }
        }

        void APFPathPlanner::create_path(std::vector<std::vector<double>> &path, double init_robot_pose, double max_path_length, size_t path_search_range)
        {
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre;
            double path_length  = 0;
            size_t range        = path_search_range;
            std::vector<Potential::FieldGrid>* field_values;
            Field& field = potential_field_;
            field_values = field.get_values();

            for (auto value : (*field_values))
            {
                if (value.states[Potential::GridInfo::IS_ROBOT])
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
            path.push_back({center_x, center_y});
            (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;

            __sort_repulsion_edges();

            double theta_pre = init_robot_pose;
            double x_pre     = path.back()[0];
            double y_pre     = path.back()[1];

            bool solving_local_minimum = false;
            bool change_weight = false;
            while ((*field_values)[pf_idx_min].states[Potential::GridInfo::IS_AROUND_GOAL] == false && 
                    path_length <= max_path_length)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path.size() > 100) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                field.get_square_index(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[Potential::GridInfo::IS_PLANNED_PATH] == true) continue;

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

                    std::vector<Potential::FieldGrid> edges_clockwise, edges_counterclockwise;
                    __get_repulsion_edges(edges_clockwise, edges_counterclockwise, center_row, center_col);




                    // path.clear();
                    // for (size_t i = 0; i < edges_clockwise.size(); i++)
                    // {
                    //     path.push_back({edges_clockwise[i].x, edges_clockwise[i].y});
                    //     (*field_values)[edges_clockwise[i].index].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;

                    // path.clear();
                    // for (size_t i = 0; i < edges_counterclockwise.size(); i++)
                    // {
                    //     path.push_back({edges_counterclockwise[i].x, edges_counterclockwise[i].y});
                    //     (*field_values)[edges_counterclockwise[i].index].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;




                    // double distance_to_goal_clockwise = std::numeric_limits<double>::infinity();
                    size_t path_length_clockwise;
                    for (path_length_clockwise = 0;path_length_clockwise < edges_clockwise.size()/2; path_length_clockwise++)
                    {
                        if (__get_smaller_potential_index(edges_clockwise[path_length_clockwise].index, J_min) != edges_clockwise[path_length_clockwise].index)
                        {
                            break;
                        }
                    }

                    size_t path_length_counterclockwise;
                    for (path_length_counterclockwise = 0;path_length_counterclockwise < edges_counterclockwise.size()/2; path_length_counterclockwise++)
                    {
                        if (__get_smaller_potential_index(edges_counterclockwise[path_length_counterclockwise].index, J_min) != edges_counterclockwise[path_length_counterclockwise].index)
                        {
                            break;
                        }
                    }
                    
                    if(path_length_clockwise < path_length_counterclockwise)
                    {
                        for (size_t i = 0; i < path_length_clockwise; i++)
                        {
                            path.push_back({edges_clockwise[i].x, edges_clockwise[i].y});
                            (*field_values)[edges_clockwise[i].index].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                        }
                        pf_idx_min = edges_clockwise[path_length_clockwise].index;
                    }
                    else
                    {
                        for (size_t i = 0; i < path_length_counterclockwise; i++)
                        {
                            path.push_back({edges_counterclockwise[i].x, edges_counterclockwise[i].y});
                            (*field_values)[edges_counterclockwise[i].index].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                        }
                        pf_idx_min = edges_counterclockwise[path_length_counterclockwise].index;
                    }

                    solving_local_minimum   = false;
                }
                
                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path.back()[0],2) + pow(py - path.back()[1],2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path.back()[1],px-path.back()[0]);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                if ((std::vector<double>){px, py} == path.end()[-1] && (std::vector<double>){px, py} == path.end()[-2]) break;

                path.push_back({px, py});
                (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                
            }
        }

        size_t APFPathPlanner::__get_smaller_potential_index(size_t centor_index, double potential_value)
        {
            std::vector<Potential::FieldGrid>* field_values;
            Field& field = potential_field_;
            field_values = field.get_values();

            size_t centor_row = (*field_values)[centor_index].row;
            size_t centor_col = (*field_values)[centor_index].col;

            if(potential_value < 0)
            {
                potential_value = 0;
            }
            
            std::vector<size_t> search_indexes;
            field.get_square_index(search_indexes, centor_row, centor_col, 1);
            if (search_indexes.empty()) return centor_index;

            double distance_from_centor_to_goal = sqrt(pow((*field_values)[centor_index].x - goal_[0],2)+pow((*field_values)[centor_index].y - goal_[1],2));

            for (auto idx : search_indexes)
            {
                if ((*field_values)[idx].value < potential_value) 
                {
                    double distance_from_idx_to_goal = sqrt(pow((*field_values)[idx].x - goal_[0],2)+pow((*field_values)[idx].y - goal_[1],2));
                    if (distance_from_idx_to_goal < distance_from_centor_to_goal)
                    {
                        return idx;
                    }
                }
            }

            return centor_index;

        }

        void APFPathPlanner::__sort_repulsion_edges()
        {
            loop_edges_.clear();
            Field& field = potential_field_;
            
            Field edge_field;
            field.info_filter(edge_field, Potential::GridInfo::IS_REPULSION_FIELD_EDGE);
            std::vector<Potential::FieldGrid>* edge_values = edge_field.get_values();
            if((*edge_values).empty()) return;

            float next_grid_distance = sqrt(2.0) * field.get_header().resolution;
            // next_grid_distance+=0.2;
            float scale = std::pow(10, 3);
            next_grid_distance =  std::ceil(next_grid_distance * scale) / scale;

            std::vector<Potential::FieldGrid> sorted_values;
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

            // std::vector<std::vector<potbot_lib::Potential::FieldGrid>> merged;
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

        void APFPathPlanner::__get_repulsion_edges(std::vector<Potential::FieldGrid>& edges_clockwise, std::vector<Potential::FieldGrid>& edges_counterclockwise, size_t row_centor, size_t col_centor)
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
            




            Field& field = potential_field_;
            size_t start_index = field.get_field_index(row_centor, col_centor);

            float min_distance = std::numeric_limits<float>::infinity();
            float start_x = field.get_value(start_index).x;
            float start_y = field.get_value(start_index).y;
            // std::vector<Potential::FieldGrid> near_edges;
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

        void APFPathPlanner::bezier(const std::vector<std::vector<double>> &path_control, std::vector<std::vector<double>> &path_interpolated)
        {
            // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/

            if (path_control.size() < 2) return;
            path_interpolated.clear();

            double x_min = path_control[0][0];
            double x_max = path_control[0][0];
            double y_min = path_control[0][1];
            double y_max = path_control[0][1];
            for(const auto& point : path_control)
            {
                double x = point[0];
                double y = point[1];
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
            }

            double n = path_control.size();

            int bezier_idx = 0;
            double inc = 1.0/double(n*10);
            double interpolate_distance_threshold = sqrt(pow(path_control[0][0] - path_control[1][0],2) + pow(path_control[0][1] - path_control[1][1],2));
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = 0;
                double y = 0;
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    double a = __nCr(n-1.0,i);
                    double b = pow(t,i);
                    double c = pow(1.0-t,n-i-1.0);
                    double x_inc = a * b * c * path_control[size_t(i)][0];
                    double y_inc = __nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)][1];
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
                    double distance = sqrt(pow(x - path_interpolated.back()[0],2) + pow(y - path_interpolated.back()[1],2));
                    if (distance > interpolate_distance_threshold) break;
                }
                
                path_interpolated.push_back({x,y});
            }
        }

        double APFPathPlanner::__nCr(double n, double r)
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