#include <potbot_lib/field.h>

namespace potbot_lib{
    namespace potential{

        Field::Field(size_t rows, size_t cols, double resolution, double origin_x, double origin_y)
        {
            initField(rows, cols, resolution, origin_x, origin_y);
        }

        // Field::Field(costmap_2d::Costmap2D* costmap)
        // {
        //     init_field( costmap->getSizeInCellsY(), costmap->getSizeInCellsX(), 
        //                 costmap->getResolution(), 
        //                 costmap->getOriginX() + costmap->getSizeInMetersX()/2, costmap->getOriginY() + costmap->getSizeInMetersY()/2);
        // }

        void Field::initField(size_t rows, size_t cols, double resolution, double origin_x, double origin_y)
        {
            values_.clear();
            origin_.x = origin_x;
            origin_.y = origin_y;


            header_.rows            = rows;
            header_.cols            = cols;

            header_.width           = resolution*(double)cols;
            header_.height          = resolution*(double)rows;
            header_.resolution      = resolution;
            
            size_t field_index      = 0;

            // header_.x_shift    = -header_.resolution/2.0 - header_.width/2.0;
            // header_.y_shift    = -header_.resolution/2.0 - header_.height/2.0;

            header_.x_shift         = -header_.width/2.0 + origin_.x;
            header_.y_shift         = -header_.height/2.0 + origin_.y;

            header_.x_min           = -header_.width/2.0 + origin_.x;
            header_.x_max           = header_.width/2.0 + origin_.x;
            header_.y_min           = -header_.height/2.0 + origin_.y;
            header_.y_max           = header_.height/2.0 + origin_.y;

            for (size_t row = 0; row < header_.rows; row++)
            {
                double y            = (double)row*header_.resolution + header_.y_shift;
                for (size_t col = 0; col < header_.cols; col++)
                {
                    double x        = (double)col*header_.resolution + header_.x_shift;
                    
                    potential::FieldGrid grid;
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

        void Field::setValues(std::vector<FieldGrid>& values)
        {
            if (values_.size() == values.size()) values_ = values;
        }

        void Field::setValue(FieldGrid value)
        {
            size_t idx = value.index;
            checkIndex(idx);
            values_[idx] = value;
        }

        void Field::searchFieldInfo(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode)
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

        void Field::searchFieldInfo(std::vector<size_t>& result, const size_t term)
        {
            searchFieldInfo(result, {term}, "or");
        }

        int Field::checkIndex(auto index)
        {
            if(index < 0 || index >= values_.size()) throw std::out_of_range("invalid index argument");
            return 0;
        }

        Point Field::getOrigin()
        {
            return origin_;
        }

        FieldHeader Field::getHeader()
        {
            return header_;
        }

        std::vector<FieldGrid>* Field::getValues()
        {
            return &values_;
        }

        FieldGrid Field::getValue(size_t index)
        {
            checkIndex(index);
            return values_[index];
        }

        FieldGrid Field::getValue(double x, double y)
        {
            return getValue(getFieldIndex(x,y));
        }

        size_t Field::getFieldIndex(double x, double y)
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
            size_t idx = getFieldIndex(row, col);
            return idx;
        }

        size_t Field::getFieldIndex(Point p)
        {
            return getFieldIndex(p.x, p.y);
        }

        size_t Field::getFieldIndex(size_t row, size_t col)
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

        std::vector<double> Field::getFieldCoordinate(size_t index)
        {
            checkIndex(index);
            std::vector<double> coord(2);
            coord[0] = values_[index].x;
            coord[1] = values_[index].y;
            return coord;
        }

        void Field::setFieldInfo(size_t index, size_t meta, bool value)
        {
            if(index < values_.size()) values_[index].states[meta] = value;
        }

        void Field::getSquareIndex(std::vector<size_t>& search_indexes, size_t centor_row, size_t centor_col, size_t range)
        {
            for (size_t row = centor_row-range; row <= centor_row+range; row++)
            {
                for (size_t col = centor_col-range; col <= centor_col+range; col++)
                {
                    if (row == centor_row && col == centor_col) continue;
                    try 
                    {
                        int pf_idx = getFieldIndex(row,col);
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

        void Field::infoFilter(Field& field, const std::vector<size_t> terms, const std::string mode)
        {
            field.values_.clear();
            std::vector<size_t> filterd_index;
            searchFieldInfo(filterd_index, terms, mode);
            for (auto idx : filterd_index)
            {
                field.values_.push_back(values_[idx]);
            }
        }

        void Field::infoFilter(Field& field, const size_t terms)
        {
            infoFilter(field, {terms}, "or");
        }
    }
}