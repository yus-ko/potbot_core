#include <potbot_lib/filter.hpp>

namespace potbot_lib
{

    namespace filter
    {

        MoveMean::MoveMean(int window_num) : window_num_(window_num)
        {
        }

        void MoveMean::setWindowNum(int num)
        {
            window_num_ = num;
        }

        void MoveMean::setData(double data)
        {
            window_vector_.push_back(data);
            if (static_cast<int>(window_vector_.size()) > window_num_)
            {
                window_vector_.pop_front();
            }
        }

        double MoveMean::mean()
        {
            if (window_vector_.empty())
            {
                return 0.0;
            }
            double sum = 0.0;
            for (const double &val : window_vector_)
            {
                sum += val;
            }
            return sum / static_cast<double>(window_vector_.size());
        }

        MoveMeanPose::MoveMeanPose(int window_num)
            : window_vectors_(6, MoveMean(window_num))
        {
        }

        void MoveMeanPose::setWindowNum(int num)
        {
            for (MoveMean &m : window_vectors_)
            {
                m.setWindowNum(num);
            }
        }

        void MoveMeanPose::setData(potbot_lib::Pose pose)
        {
            window_vectors_[0].setData(pose.position.x);
            window_vectors_[1].setData(pose.position.y);
            window_vectors_[2].setData(pose.position.z);
            window_vectors_[3].setData(pose.rotation.x);
            window_vectors_[4].setData(pose.rotation.y);
            window_vectors_[5].setData(pose.rotation.z);
        }

        potbot_lib::Pose MoveMeanPose::mean()
        {
            potbot_lib::Pose result;
            result.position.x = window_vectors_[0].mean();
            result.position.y = window_vectors_[1].mean();
            result.position.z = window_vectors_[2].mean();
            result.rotation.x = window_vectors_[3].mean();
            result.rotation.y = window_vectors_[4].mean();
            result.rotation.z = window_vectors_[5].mean();
            return result;
        }

        LowPass::LowPass(double filter_coefficient)
            : filter_coefficient_(filter_coefficient)
        {
        }

        void LowPass::setFilterCoefficient(double val)
        {
            filter_coefficient_ = val;
        }

        void LowPass::setData(double data)
        {
            input_ = data;
        }

        double LowPass::filter()
        {
            double output = (1.0 - filter_coefficient_) * input_ + filter_coefficient_ * prev_output_;
            prev_output_ = output;
            return output;
        }

        LowPassPose::LowPassPose(double filter_coefficient)
            : data_vector_(6, LowPass(filter_coefficient))
        {
        }

        void LowPassPose::setFilterCoefficient(double val)
        {
            for (LowPass &lp : data_vector_)
            {
                lp.setFilterCoefficient(val);
            }
        }

        void LowPassPose::setData(potbot_lib::Pose pose)
        {
            data_vector_[0].setData(pose.position.x);
            data_vector_[1].setData(pose.position.y);
            data_vector_[2].setData(pose.position.z);
            data_vector_[3].setData(pose.rotation.x);
            data_vector_[4].setData(pose.rotation.y);
            data_vector_[5].setData(pose.rotation.z);
        }

        potbot_lib::Pose LowPassPose::filter()
        {
            potbot_lib::Pose result;
            result.position.x = data_vector_[0].filter();
            result.position.y = data_vector_[1].filter();
            result.position.z = data_vector_[2].filter();
            result.rotation.x = data_vector_[3].filter();
            result.rotation.y = data_vector_[4].filter();
            result.rotation.z = data_vector_[5].filter();
            return result;
        }

    }
}
