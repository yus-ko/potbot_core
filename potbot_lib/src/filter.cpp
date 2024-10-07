#include <potbot_lib/filter.h>

namespace potbot_lib{
    namespace filter{
        MoveMean::MoveMean(int window_num)
        {
            window_num_ = window_num;
        }

        void MoveMean::setWindowNum(int num)
        {
            if (num != window_num_)
            {
                window_num_ = num;
                window_vector_.clear();
            }
        }

        void MoveMean::setData(double data)
        {
            window_vector_.push_back(data);
            while (window_vector_.size() > window_num_)
            {
                window_vector_.pop_front();
            }
        }

        double MoveMean::mean()
        {
            double num = static_cast<double>(window_vector_.size());
            double sum = 0;
            for (const auto v:window_vector_)
            {
                sum+=v;
            }
            return sum/num;
        }

        MoveMeanPose::MoveMeanPose(int window_num)
        {
            std::deque<MoveMean> tmp(12, MoveMean(window_num));
            window_vectors_ = tmp;
        }

        void MoveMeanPose::setWindowNum(int num)
        {
            for (auto& v:window_vectors_)
            {
                v.setWindowNum(num);
            }
        }

        void MoveMeanPose::setData(Pose pose)
        {
            window_vectors_[0].setData(pose.position.x);
            window_vectors_[1].setData(pose.position.y);
            window_vectors_[2].setData(pose.position.z);
            Eigen::Matrix3d R = pose.rotation.to_rotation();
            window_vectors_[3].setData(R(0,0));
            window_vectors_[4].setData(R(0,1));
            window_vectors_[5].setData(R(0,2));
            window_vectors_[6].setData(R(1,0));
            window_vectors_[7].setData(R(1,1));
            window_vectors_[8].setData(R(1,2));
            window_vectors_[9].setData(R(2,0));
            window_vectors_[10].setData(R(2,1));
            window_vectors_[11].setData(R(2,2));
        }

        Pose MoveMeanPose::mean()
        {
            Pose pose;
            pose.position.x = window_vectors_[0].mean();
            pose.position.y = window_vectors_[1].mean();
            pose.position.z = window_vectors_[2].mean();
            Eigen::Matrix3d R;
            R <<    window_vectors_[3].mean(), window_vectors_[4].mean(), window_vectors_[5].mean(),
                    window_vectors_[6].mean(), window_vectors_[7].mean(), window_vectors_[8].mean(),
                    window_vectors_[9].mean(), window_vectors_[10].mean(), window_vectors_[11].mean();
            pose.rotation = Point(R);
            return pose;
        }

        LowPass::LowPass(double filter_coefficient)
        {
            filter_coefficient_ = filter_coefficient;
        }

        void LowPass::setFilterCoefficient(double val)
        {
            filter_coefficient_ = val;
        }

        void LowPass::setData(double data)
        {
            if (data_.size() < 2)
            {
                data_.push_back(data);
            }
            else
            {
                data_[0] = data_[1];
                data_[1] = data;
            }
            
        }
        double LowPass::filter()
        {
            if (data_.size() == 1)
            {
                return data_[0];
            }
            else if (data_.size() >= 2)
            {
                data_[1] = filter_coefficient_*data_[1] + (1.0-filter_coefficient_)*data_[0];
                return data_[1];
            }
        }

        LowPassPose::LowPassPose(double filter_coefficient)
        {
            std::vector<LowPass> tmp(12, LowPass(filter_coefficient));
            data_vector_ = tmp;
        }

        void LowPassPose::setFilterCoefficient(double val)
        {
            for (auto& v:data_vector_)
            {
                v.setFilterCoefficient(val);
            }
        }

        void LowPassPose::setData(Pose pose)
        {
            data_vector_[0].setData(pose.position.x);
            data_vector_[1].setData(pose.position.y);
            data_vector_[2].setData(pose.position.z);
            Eigen::Matrix3d R = pose.rotation.to_rotation();
            data_vector_[3].setData(R(0,0));
            data_vector_[4].setData(R(0,1));
            data_vector_[5].setData(R(0,2));
            data_vector_[6].setData(R(1,0));
            data_vector_[7].setData(R(1,1));
            data_vector_[8].setData(R(1,2));
            data_vector_[9].setData(R(2,0));
            data_vector_[10].setData(R(2,1));
            data_vector_[11].setData(R(2,2));
        }

        Pose LowPassPose::filter()
        {
            Pose pose;
            pose.position.x = data_vector_[0].filter();
            pose.position.y = data_vector_[1].filter();
            pose.position.z = data_vector_[2].filter();
            Eigen::Matrix3d R;
            R <<    data_vector_[3].filter(), data_vector_[4].filter(), data_vector_[5].filter(),
                    data_vector_[6].filter(), data_vector_[7].filter(), data_vector_[8].filter(),
                    data_vector_[9].filter(), data_vector_[10].filter(), data_vector_[11].filter();
            pose.rotation = Point(R);
            return pose;
        }
    }
}