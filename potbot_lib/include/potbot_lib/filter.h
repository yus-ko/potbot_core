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

        class LowPass
        {
            protected:
                double filter_coefficient_ = 0.5;
                std::vector<double> data_; 

            public:
                LowPass(double filter_coefficient = 0.5);
                ~LowPass(){};

                void setFilterCoefficient(double val);
                void setData(double data);
                double filter();
        };

        class LowPassPose : private LowPass
        {
            protected:
                std::vector<LowPass> data_vector_; 

            public:
                LowPassPose(double filter_coefficient = 0.5);
                ~LowPassPose(){};

                void setFilterCoefficient(double val);
                void setData(Pose pose);
                Pose filter();
        };
    }
}

#endif	// H_POTBOT_LIB_FILTER_