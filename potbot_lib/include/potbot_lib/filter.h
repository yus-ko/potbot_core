#ifndef H_POTBOT_LIB_FILTER_
#define H_POTBOT_LIB_FILTER_

#include <potbot_lib/utility.h>
#include <iostream>
#include <deque>
#include <vector>

namespace potbot_lib
{
    namespace filter
    {
        class MoveMean
        {
            protected:
                int window_num_ = 10;
                std::deque<double> window_vector_; 

            public:
                MoveMean(int window_num = 10);
                ~MoveMean(){};

                void setWindowNum(int num);
                void setData(double data);
                double mean();
        };

        class MoveMeanPose : private MoveMean
        {
            protected:
                std::deque<MoveMean> window_vectors_; 

            public:
                MoveMeanPose(int window_num = 10);
                ~MoveMeanPose(){};

                void setWindowNum(int num);
                void setData(Pose pose);
                Pose mean();
        };
    }
}

#endif	// H_POTBOT_LIB_FILTER_