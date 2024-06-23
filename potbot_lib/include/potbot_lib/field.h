#ifndef H_POTBOT_LIB_FIELD_
#define H_POTBOT_LIB_FIELD_

#include <potbot_lib/utility.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace potbot_lib{

    namespace potential{

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

                Point origin_;
                FieldHeader header_;
                std::vector<FieldGrid> values_;

            public:
                Field(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double origin_x = 0.0, double origin_y = 0.0);
                // Field(costmap_2d::Costmap2D* costmap);
                ~Field(){};

                void initField(size_t rows = 3, size_t cols = 3, double resolution = 1.0, double origin_x = 0.0, double origin_y = 0.0);

                void setValues(std::vector<FieldGrid>& values);
                void setValue(FieldGrid value);

                void setFieldInfo(size_t index = 0, size_t meta = potential::GridInfo::IS_OBSTACLE, bool value = false);
                void searchFieldInfo(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode = "and");
                void searchFieldInfo(std::vector<size_t>& result, const size_t term);

                int checkIndex(auto index);

                Point getOrigin();
                FieldHeader getHeader();
                std::vector<FieldGrid>* getValues();
                FieldGrid getValue(size_t index = 0);
                FieldGrid getValue(double x = 0, double y = 0);

                size_t getFieldIndex(double x = 0, double y = 0);
                size_t getFieldIndex(Point p);
                size_t getFieldIndex(size_t row = 0, size_t col = 0);
                std::vector<double> getFieldCoordinate(size_t index = 0);

                void getSquareIndex(std::vector<size_t>& search_indexes, size_t centor_row = 0, size_t centor_col = 0, size_t range = 1);

                void infoFilter(Field& field, const std::vector<size_t> terms, const std::string mode = "and");
                void infoFilter(Field& field, const size_t term);
        };
    }
}

#endif	// H_POTBOT_LIB_FIELD_