#ifndef _H_UNSCENTEDKALMANFILTER_
#define _H_UNSCENTEDKALMANFILTER_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <potbot_lib/DiffDriveController.h>

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
            std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> __U_transform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt); 

        public:
            UnscentedKalmanFilter(ModelFunction model_func_system, ModelFunction model_func_observ, 
                Eigen::MatrixXd noise_covariance_system, Eigen::MatrixXd noise_covariance_observ, 
                Eigen::MatrixXd error_covariance_ini, Eigen::VectorXd estimate_state_ini);
            ~UnscentedKalmanFilter();

            std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd y, double dt);

            void get_odom_state(nav_msgs::Odometry& odom_msg);

    };
}

#endif	// _H_UNSCENTEDKALMANFILTER_