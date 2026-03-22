#ifndef HPP_POTBOT_LIB_FILTER_
#define HPP_POTBOT_LIB_FILTER_

#include <potbot_lib/utility.hpp>
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
            ~MoveMean() {};
            void setWindowNum(int num);
            void setData(double data);
            double mean();
        };

        class MoveMeanPose
        {
        protected:
            std::vector<MoveMean> window_vectors_;

        public:
            MoveMeanPose(int window_num = 10);
            ~MoveMeanPose() {};
            void setWindowNum(int num);
            void setData(potbot_lib::Pose pose);
            potbot_lib::Pose mean();
        };

        class LowPass
        {
        protected:
            double filter_coefficient_ = 0.5;
            double input_ = 0.0;
            double prev_output_ = 0.0;

        public:
            LowPass(double filter_coefficient = 0.5);
            ~LowPass() {};
            void setFilterCoefficient(double val);
            void setData(double data);
            double filter();
        };

        class LowPassPose
        {
        protected:
            std::vector<LowPass> data_vector_;

        public:
            LowPassPose(double filter_coefficient = 0.5);
            ~LowPassPose() {};
            void setFilterCoefficient(double val);
            void setData(potbot_lib::Pose pose);
            potbot_lib::Pose filter();
        };

    }
}

#endif // HPP_POTBOT_LIB_FILTER_
