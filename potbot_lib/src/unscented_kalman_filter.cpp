#include <potbot_lib/unscented_kalman_filter.hpp>

namespace potbot_lib
{

    UnscentedKalmanFilter::UnscentedKalmanFilter()
    {
        xhat_ = Eigen::VectorXd::Zero(1);
        P_ = Eigen::MatrixXd::Identity(1, 1);
        Q_ = Eigen::MatrixXd::Identity(1, 1);
        R_ = Eigen::MatrixXd::Identity(1, 1);
    }

    void UnscentedKalmanFilter::setStateSize(int n)
    {
        xhat_ = Eigen::VectorXd::Zero(n);
        P_ = Eigen::MatrixXd::Identity(n, n);
        Q_ = Eigen::MatrixXd::Identity(n, n);
    }

    void UnscentedKalmanFilter::setObservationSize(int m)
    {
        R_ = Eigen::MatrixXd::Identity(m, m);
    }

    std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd>
    UnscentedKalmanFilter::uTransform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt)
    {
        int n = static_cast<int>(xm.size());
        int sigma_count = 2 * n + 1;

        Eigen::VectorXd w(sigma_count);
        w(0) = kappa_ / (n + kappa_);
        double w_rest = 1.0 / (2.0 * (n + kappa_));
        for (int i = 1; i < sigma_count; ++i)
        {
            w(i) = w_rest;
        }

        Eigen::LLT<Eigen::MatrixXd> llt((n + kappa_) * Pxx);
        if (llt.info() != Eigen::Success)
        {
            return std::make_tuple(false, xm, Pxx, Eigen::MatrixXd::Zero(n, 1));
        }
        Eigen::MatrixXd L = llt.matrixL();

        Eigen::MatrixXd X(n, sigma_count);
        X.col(0) = xm;
        for (int i = 0; i < n; ++i)
        {
            X.col(i + 1) = xm + L.col(i);
            X.col(n + i + 1) = xm - L.col(i);
        }

        // col(0)の二重評価を避けるため先に評価してからループ
        Eigen::VectorXd y0 = f_ut(X.col(0), dt);
        Eigen::MatrixXd Y(y0.size(), sigma_count);
        Y.col(0) = y0;
        for (int i = 1; i < sigma_count; ++i)
        {
            Y.col(i) = f_ut(X.col(i), dt);
        }

        int m = static_cast<int>(Y.rows());

        Eigen::VectorXd ym = Eigen::VectorXd::Zero(m);
        for (int i = 0; i < sigma_count; ++i)
        {
            ym += w(i) * Y.col(i);
        }

        Eigen::MatrixXd Pyy = R_;
        Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(n, m);
        for (int i = 0; i < sigma_count; ++i)
        {
            Eigen::VectorXd dy = Y.col(i) - ym;
            Pyy += w(i) * dy * dy.transpose();
            Pxy += w(i) * (X.col(i) - xm) * dy.transpose();
        }

        Eigen::MatrixXd G = Pxy * Pyy.inverse();

        return std::make_tuple(true, ym, Pyy, G);
    }

    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd>
    UnscentedKalmanFilter::update(Eigen::VectorXd y, double dt)
    {
        auto [pred_ok, xhat_pred, P_pred, K_pred] = uTransform(f_model_, xhat_, P_ + Q_, dt);
        if (!pred_ok)
        {
            return std::make_tuple(xhat_, P_, Eigen::MatrixXd::Zero(xhat_.size(), y.size()));
        }

        auto [upd_ok, ym, Pyy, G] = uTransform(h_model_, xhat_pred, P_pred, dt);
        if (!upd_ok)
        {
            return std::make_tuple(xhat_, P_, Eigen::MatrixXd::Zero(xhat_.size(), y.size()));
        }

        xhat_ = xhat_pred + G * (y - ym);
        P_ = P_pred - G * Pyy * G.transpose();

        return std::make_tuple(xhat_, P_, G);
    }

} // namespace potbot_lib
