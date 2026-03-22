#include <potbot_lib/kalman_filter.hpp>

namespace potbot_lib
{

    KalmanFilter::KalmanFilter()
    {
        A_ = Eigen::MatrixXd::Identity(1, 1);
        C_ = Eigen::MatrixXd::Identity(1, 1);
        Q_ = Eigen::MatrixXd::Identity(1, 1);
        R_ = Eigen::MatrixXd::Identity(1, 1);
        P_ = Eigen::MatrixXd::Identity(1, 1);
        K_ = Eigen::MatrixXd::Zero(1, 1);
        xhat_ = Eigen::VectorXd::Zero(1);
        z_ = Eigen::VectorXd::Zero(1);
    }

    void KalmanFilter::initialize()
    {
        int n = xhat_.size();
        P_ = Eigen::MatrixXd::Identity(n, n);
        K_ = Eigen::MatrixXd::Zero(n, C_.rows());
        xhat_ = Eigen::VectorXd::Zero(n);
        z_ = Eigen::VectorXd::Zero(C_.rows());
    }

    void KalmanFilter::setA(Eigen::MatrixXd mat)
    {
        A_ = mat;
        int n = mat.rows();
        xhat_ = Eigen::VectorXd::Zero(n);
        P_ = Eigen::MatrixXd::Identity(n, n);
        Q_ = Eigen::MatrixXd::Identity(n, n);
        K_ = Eigen::MatrixXd::Zero(n, C_.rows());
        z_ = Eigen::VectorXd::Zero(C_.rows());
    }

    void KalmanFilter::setC(Eigen::MatrixXd mat)
    {
        C_ = mat;
        int m = mat.rows();
        R_ = Eigen::MatrixXd::Identity(m, m);
        K_ = Eigen::MatrixXd::Zero(A_.rows(), m);
        z_ = Eigen::VectorXd::Zero(m);
    }

    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd>
    KalmanFilter::update(Eigen::VectorXd data, double /*dt*/)
    {
        z_ = data;

        int n = xhat_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

        Eigen::VectorXd xhat_pred = A_ * xhat_;
        Eigen::MatrixXd P_pred = A_ * P_ * A_.transpose() + Q_;

        Eigen::MatrixXd S = C_ * P_pred * C_.transpose() + R_;
        K_ = P_pred * C_.transpose() * S.inverse();
        xhat_ = xhat_pred + K_ * (data - C_ * xhat_pred);
        P_ = (I - K_ * C_) * P_pred;

        return std::make_tuple(xhat_, P_, K_);
    }

} // namespace potbot_lib
