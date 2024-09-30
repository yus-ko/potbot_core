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
            std::deque<MoveMean> tmp(6, MoveMean(window_num));
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
            window_vectors_[3].setData(pose.rotation.x);
            window_vectors_[4].setData(pose.rotation.y);
            window_vectors_[5].setData(pose.rotation.z);
        }

        Pose MoveMeanPose::mean()
        {
            Pose pose;
            pose.position.x = window_vectors_[0].mean();
            pose.position.y = window_vectors_[1].mean();
            pose.position.z = window_vectors_[2].mean();
            pose.rotation.x = window_vectors_[3].mean();
            pose.rotation.y = window_vectors_[4].mean();
            pose.rotation.z = window_vectors_[5].mean();
            return pose;
        }
    }
}