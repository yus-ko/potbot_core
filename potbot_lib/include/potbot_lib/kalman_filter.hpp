#ifndef HPP_POTBOT_LIB_KALMAN_FILTER_
#define HPP_POTBOT_LIB_KALMAN_FILTER_

#include <Eigen/Dense>
#include <tuple>

namespace potbot_lib
{
    class KalmanFilter
    {
    private:
        Eigen::MatrixXd A_;    // 状態遷移行列
        Eigen::MatrixXd C_;    // 観測行列
        Eigen::MatrixXd Q_;    // プロセスノイズ共分散
        Eigen::MatrixXd R_;    // 観測ノイズ共分散
        Eigen::MatrixXd P_;    // 誤差共分散行列
        Eigen::MatrixXd K_;    // カルマンゲイン
        Eigen::VectorXd xhat_; // 推定状態
        Eigen::VectorXd z_;    // 観測値

    public:
        KalmanFilter();
        ~KalmanFilter() {};

        void initialize();

        // 状態更新: return (xhat, P, K)
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd data, double dt);

        void setA(Eigen::MatrixXd mat);
        void setC(Eigen::MatrixXd mat);

        inline Eigen::VectorXd get_xhat() { return xhat_; }
        inline Eigen::VectorXd get_z() { return z_; }
        inline Eigen::MatrixXd get_K() { return K_; }
        inline Eigen::MatrixXd get_P() { return P_; }
    };
} // namespace potbot_lib

#endif // HPP_POTBOT_LIB_KALMAN_FILTER_
