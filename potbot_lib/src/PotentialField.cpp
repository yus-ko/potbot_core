#include <potbot_lib/PotentialField.h>

namespace potbot_lib{
    namespace Potential{

        Field::Field(size_t rows, size_t cols, double resolution, double origin_x, double origin_y)
        {
            init_field(rows, cols, resolution, origin_x, origin_y);
        }
        Field::~Field(){}

        void Field::init_field(size_t rows, size_t cols, double resolution, double origin_x, double origin_y)
        {
            values_.clear();

            header_.rows            = rows;
            header_.cols            = cols;

            header_.width           = resolution*(double)cols;
            header_.height          = resolution*(double)rows;
            header_.resolution      = resolution;
            
            size_t field_index      = 0;

            // header_.x_shift    = -header_.resolution/2.0 - header_.width/2.0;
            // header_.y_shift    = -header_.resolution/2.0 - header_.height/2.0;

            header_.x_shift         = -header_.width/2.0 + origin_x;
            header_.y_shift         = -header_.height/2.0 + origin_y;

            header_.x_min           = -header_.width/2.0 + origin_x;
            header_.x_max           = header_.width/2.0 + origin_x;
            header_.y_min           = -header_.height/2.0 + origin_y;
            header_.y_max           = header_.height/2.0 + origin_y;

            for (size_t row = 0; row < header_.rows; row++)
            {
                double y            = (double)row*header_.resolution + header_.y_shift;
                for (size_t col = 0; col < header_.cols; col++)
                {
                    double x        = (double)col*header_.resolution + header_.x_shift;
                    
                    Potential::FieldGrid grid;
                    grid.index      = field_index;
                    grid.x          = x;
                    grid.y          = y;
                    grid.row        = row;
                    grid.col        = col;
                    values_.push_back(grid);
                    field_index++;
                }
            }
        }

        void Field::set_values(std::vector<FieldGrid>& values)
        {
            if (values_.size() == values.size()) values_ = values;
        }

        void Field::set_value(FieldGrid value)
        {
            size_t idx = value.index;
            check_index(idx);
            values_[idx] = value;
        }

        void Field::search_field_info(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode)
        {
            if (terms.empty()) return;

            if (terms.size() == 1)
            {
                for (auto value : values_)
                {
                    if (value.states[terms[0]]) result.push_back(value.index);
                }
            }
            else if (mode == "or")
            {
                for (auto value : values_)
                {
                    for (auto term : terms)
                    {
                        if (value.states[term])
                        {
                            result.push_back(value.index);
                            break;
                        }
                    }
                    
                }
            }
            else if (mode == "and")
            {
                for (auto value : values_)
                {
                    bool logic_and = true;
                    for (auto term : terms)
                    {
                        logic_and *= value.states[term];
                        if (!logic_and) break;
                    }
                    if (logic_and) result.push_back(value.index);
                }
            }
        }

        void Field::search_field_info(std::vector<size_t>& result, const size_t term)
        {
            search_field_info(result, {term}, "or");
        }

        int Field::check_index(auto index)
        {
            if(index < 0 || index >= values_.size()) throw std::out_of_range("invalid index argument");
            return 0;
        }

        FieldHeader Field::get_header()
        {
            return header_;
        }

        std::vector<FieldGrid>* Field::get_values()
        {
            return &values_;
        }

        FieldGrid Field::get_value(size_t index)
        {
            check_index(index);
            return values_[index];
        }

        FieldGrid Field::get_value(double x, double y)
        {
            return get_value(get_field_index(x,y));
        }

        size_t Field::get_field_index(double x, double y)
        {
            if(x > header_.x_max || x < header_.x_min)
            {
                throw std::out_of_range("invalid coordinate x argument");
            }
            else if (y > header_.y_max || y < header_.y_min)
            {
                throw std::out_of_range("invalid coordinate y argument");
            }
            
            size_t col = (x - header_.x_shift)/header_.resolution;
            size_t row = (y - header_.y_shift)/header_.resolution;
            size_t idx = get_field_index(row, col);
            return idx;
        }

        size_t Field::get_field_index(size_t row, size_t col)
        {
            if (row >= header_.rows)
            {

                throw std::out_of_range("invalid row argument");
            }
            else if (col >= header_.cols)
            {
                throw std::out_of_range("invalid col argument");
            }
            size_t idx = row*header_.cols + col;
            return idx;
        }

        std::vector<double> Field::get_field_coordinate(size_t index)
        {
            check_index(index);
            std::vector<double> coord(2);
            coord[0] = values_[index].x;
            coord[1] = values_[index].y;
            return coord;
        }

        void Field::set_field_info(size_t index, size_t meta, bool value)
        {
            if(index < values_.size()) values_[index].states[meta] = value;
        }

        void Field::get_square_index(std::vector<size_t>& search_indexes, size_t centor_row, size_t centor_col, size_t range)
        {
            for (size_t row = centor_row-range; row <= centor_row+range; row++)
            {
                for (size_t col = centor_col-range; col <= centor_col+range; col++)
                {
                    if (row == centor_row && col == centor_col) continue;
                    try 
                    {
                        int pf_idx = get_field_index(row,col);
                        search_indexes.push_back(pf_idx);
                    }
                    catch(std::out_of_range& oor) 
                    {
                        search_indexes.clear();
                        return;
                    }
                    
                }
            }
        }

        void Field::to_pcl2(sensor_msgs::PointCloud2& pcl_msg)
        {
            // std::vector<pcl::PointXYZ> を作成
            std::vector<pcl::PointXYZ> pointVector;
            for (auto value : values_)
            {
                double x = value.x;
                double y = value.y;
                double z = value.value;
                pcl::PointXYZ point(x,y,z);
                pointVector.push_back(point);
            }

            // std::vector<pcl::PointXYZ> を pcl::PointCloud に変換
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pclPointCloud->points.resize(pointVector.size());
            for (size_t i = 0; i < pointVector.size(); ++i) {
                pclPointCloud->points[i] = pointVector[i];
            }

            // pcl::PointCloud を sensor_msgs::PointCloud2 に変換
            pcl::toROSMsg(*pclPointCloud, pcl_msg);
        }

        void Field::info_filter(Field& field, const std::vector<size_t> terms, const std::string mode)
        {
            field.values_.clear();
            std::vector<size_t> filterd_index;
            search_field_info(filterd_index, terms, mode);
            for (auto idx : filterd_index)
            {
                field.values_.push_back(values_[idx]);
            }
        }

        void Field::info_filter(Field& field, const size_t terms)
        {
            info_filter(field, {terms}, "or");
        }
    }
}

namespace potbot_lib{

    APF::APF(size_t rows, size_t cols, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field, double field_origin_x, double field_origin_y) : 
    Potential::Field::Field(rows, cols, resolution)
    {
        weight_attraction_field_                = weight_attraction_field;
        weight_repulsion_field_                 = weight_repulsion_field;
        distance_threshold_repulsion_field_     = distance_threshold_repulsion_field;
        init_potential_field(rows, cols, resolution, field_origin_x, field_origin_y);
    }
    APF::~APF(){}

    void APF::init_potential_field(size_t rows, size_t cols, double resolution, double field_origin_x, double field_origin_y)
    {
        potential_field_.init_field(rows, cols, resolution, field_origin_x, field_origin_y);
    }

    void APF::set_goal(size_t index)
    {
        potential_field_.set_field_info(index, Potential::GridInfo::IS_GOAL, true);
    }

    void APF::set_robot(size_t index)
    {
        potential_field_.set_field_info(index, Potential::GridInfo::IS_ROBOT, true);
    }

    void APF::set_obstacle(size_t index)
    {
        potential_field_.set_field_info(index, Potential::GridInfo::IS_OBSTACLE, true);
    }

    void APF::set_goal(double x, double y)
    {
        goal_ = {x,y};
        try 
        {
            set_goal(potential_field_.get_field_index(x,y));
        }
        catch(...){}
        
    }

    void APF::set_robot(double x, double y)
    {
        robot_ = {x,y};
        try 
        {
            set_robot(potential_field_.get_field_index(x,y));
        }
        catch(...){}
    }

    void APF::set_obstacle(double x, double y)
    {
        obstacles_.push_back({x,y});
        try 
        {
            set_obstacle(potential_field_.get_field_index(x,y));
        }
        catch(...){}
    }

    void APF::set_obstacle(const Eigen::Vector2d& vec)
    {
        set_obstacle(vec(0), vec(1));
    }

    void APF::set_obstacle(const visualization_msgs::Marker& obs)
    {
        // return;
        double origin_x = obs.pose.position.x;
        double origin_y = obs.pose.position.y;
        double origin_th = utility::get_Yaw(obs.pose.orientation);
        double res = header_.resolution*5;

        Eigen::MatrixXd vertexes;
        if (obs.type == visualization_msgs::Marker::CUBE)
        {
            double width = obs.scale.x;
            double height = obs.scale.y;

            Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
            Eigen::MatrixXd translation(4,2);
            translation <<  origin_x, origin_y,
                            origin_x, origin_y,
                            origin_x, origin_y,
                            origin_x, origin_y;
            Eigen::MatrixXd origin_vertexes(4,2);
            origin_vertexes <<  -width/2,   -height/2,
                                -width/2,   height/2,
                                width/2,    height/2,
                                width/2,    -height/2;

            vertexes = rotation*origin_vertexes.transpose() + translation.transpose();
            
        }
        else if (obs.type == visualization_msgs::Marker::SPHERE)
        {
            double width = obs.scale.x;
            double height = obs.scale.y;

            Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
            Eigen::Vector2d translation;
            translation <<  origin_x, origin_y;
            Eigen::MatrixXd origin_vertexes(4,2);
            
            size_t vertex_num = 2*M_PI/res;
            vertexes.resize(2,vertex_num);
            for (size_t i = 0; i < vertex_num; i++)
            {
                double t = 2 * M_PI * i / vertex_num;
                double x = width/2 * cos(t);
                double y = height/2 * sin(t);
                Eigen::Vector2d p;
                p<< x,y;
                vertexes.col(i) = rotation*p + translation;
            }
        }

        for (size_t i = 0; i < vertexes.cols(); i++)
        {
            // std::cout<<vertexes.row(i)<<std::endl;
            set_obstacle(vertexes.col(i));
        }
    }

    void APF::set_obstacle(const std::vector<visualization_msgs::Marker>& obs)
    {
        for (const auto& o:obs)
        {
            set_obstacle(o);
        }
    }
    void APF::set_obstacle(const geometry_msgs::Point& obs)
    {
        set_obstacle(obs.x, obs.y);
    }

    void APF::set_obstacle(const std::vector<geometry_msgs::Point>& obs)
    {
        for (const auto& o:obs)
        {
            set_obstacle(o);
        }
    }

    void APF::set_obstacle(const geometry_msgs::PointStamped& obs)
    {
        set_obstacle(obs.point);
    }

    void APF::set_obstacle(const std::vector<geometry_msgs::PointStamped>& obs)
    {
        for (const auto& o:obs)
        {
            set_obstacle(o);
        }
    }

    void APF::set_obstacle(const geometry_msgs::Pose& obs)
    {
        set_obstacle(obs.position);
    }

    void APF::set_obstacle(const std::vector<geometry_msgs::Pose>& obs)
    {
        for (const auto& o:obs)
        {
            set_obstacle(o);
        }
    }

    void APF::set_obstacle(const geometry_msgs::PoseStamped& obs)
    {
        set_obstacle(obs.pose);
    }

    void APF::set_obstacle(const std::vector<geometry_msgs::PoseStamped>& obs)
    {
        for (const auto& o:obs)
        {
            set_obstacle(o);
        }
    }

    void APF::get_attraction_field(Potential::Field& field)
    {
        field = potential_field_;
        std::vector<Potential::FieldGrid>* potential_values;
        potential_values = field.get_values();
        for(auto& value : (*potential_values))
        {
            value.value = value.attraction;
        }
    }

    void APF::get_repulsion_field(Potential::Field& field)
    {
        field = potential_field_;
        std::vector<Potential::FieldGrid>* potential_values;
        potential_values = field.get_values();
        for(auto& value : (*potential_values))
        {
            value.value = value.repulsion;
        }
    }

    void APF::get_potential_field(Potential::Field& field)
    {
        field = potential_field_;
    }

    std::vector<double> APF::get_goal()
    {
        return goal_;
    }
    
    std::vector<double> APF::get_robot()
    {
        return robot_;
    }
    
    std::vector<std::vector<double>> APF::get_obstacles()
    {
        return obstacles_;
    }

    void APF::create_potential_field()
    {
        double weight_attraction_field = weight_attraction_field_;
        double distance_threshold_repulsion_field   = distance_threshold_repulsion_field_;
        double weight_repulsion_field               = weight_repulsion_field_;

        std::vector<Potential::FieldGrid>* potential_values;
        potential_values = potential_field_.get_values();
        for(auto& value : (*potential_values))
        {

            double x                    = value.x;
            double y                    = value.y;

            double distance_to_goal     = sqrt(pow(x - goal_[0],2)+pow(y - goal_[1],2));
            double attraction_value     = 0.5 * weight_attraction_field * pow(distance_to_goal, 2);
            value.attraction            = attraction_value;

            if (distance_to_goal < 0.3) value.states[Potential::GridInfo::IS_AROUND_GOAL] = true;



            double repulsion_value              = 0;
            for(auto& coord_obstacle : obstacles_)
            {
                double x_obstacle               = coord_obstacle[0];
                double y_obstacle               = coord_obstacle[1];
                double distance_to_obstacle     = sqrt(pow(x-x_obstacle,2) + pow(y-y_obstacle,2));
                if (distance_to_obstacle        <= distance_threshold_repulsion_field)
                {
                    value.states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE] = true;
                    repulsion_value += 0.5 * weight_repulsion_field * pow(1.0/(distance_to_obstacle + 1e-100) - 1.0/(distance_threshold_repulsion_field + 1e-100), 2);
                }
            }
            value.repulsion                     = repulsion_value;

            value.potential                     = value.attraction + value.repulsion;
            value.value                         = value.potential;
            
        }

        for(auto& value : (*potential_values))
        {

            if (value.states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
            {
                bool local_minimum              = true;
                std::vector<size_t> serch_indexes;
                potential_field_.get_square_index(serch_indexes, value.row, value.col, 1);
                for (auto idx : serch_indexes)
                {
                    try 
                    {
                        if ((*potential_values)[idx].value < value.value)
                        {
                            local_minimum   = false;
                        }

                        if ((*potential_values)[idx].states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE] == false)
                        {
                            value.states[Potential::GridInfo::IS_REPULSION_FIELD_EDGE] = true;
                        }
                    }
                    catch(std::out_of_range& oor) 
                    {
                        continue;
                    }
                }

                if(local_minimum && !value.states[Potential::GridInfo::IS_GOAL])
                {
                    value.states[Potential::GridInfo::IS_LOCAL_MINIMUM] = true;
                }
            }
        }
    }

}