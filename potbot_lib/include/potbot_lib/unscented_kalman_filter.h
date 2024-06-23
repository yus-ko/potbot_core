#ifndef H_UNSCENTEDKALMANFILTER_
#define H_UNSCENTEDKALMANFILTER_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace potbot_lib{

    class UnscentedKalmanFilter{
        private:
            // 引数として受け取る関数の型
            typedef Eigen::VectorXd (*ModelFunction)(Eigen::VectorXd, double);

            ModelFunction f_;            //システムモデル関数
            ModelFunction h_;            //観測モデル関数

            Eigen::VectorXd xhat_;       //推定状態ベクトル
            Eigen::MatrixXd P_;          //推定誤差共分散行列
            Eigen::MatrixXd Q_;          //観測ノイズ共分散行列
            Eigen::MatrixXd R_;          //システムノイズ共分散行列

            // U変換(unscented transform)
            std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> uTransform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt); 

        public:
            UnscentedKalmanFilter(ModelFunction model_func_system, ModelFunction model_func_observ, 
                Eigen::MatrixXd noise_covariance_system, Eigen::MatrixXd noise_covariance_observ, 
                Eigen::MatrixXd error_covariance_ini, Eigen::VectorXd estimate_state_ini);
            ~UnscentedKalmanFilter();

            std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd y, double dt);

            void getOdomState(nav_msgs::Odometry& odom_msg);

    };
}

#endif	// H_UNSCENTEDKALMANFILTER_