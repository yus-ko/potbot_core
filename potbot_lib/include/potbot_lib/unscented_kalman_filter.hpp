#ifndef HPP_POTBOT_LIB_UNSCENTED_KALMAN_FILTER_
#define HPP_POTBOT_LIB_UNSCENTED_KALMAN_FILTER_

#include <Eigen/Dense>
#include <functional>
#include <tuple>

namespace potbot_lib
{
    using ModelFunction = std::function<Eigen::VectorXd(Eigen::VectorXd, double)>;

    class UnscentedKalmanFilter
    {
    private:
        double kappa_ = 0;
        Eigen::VectorXd xhat_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
        ModelFunction f_model_;
        ModelFunction h_model_;

    public:
        UnscentedKalmanFilter();
        ~UnscentedKalmanFilter() {};

        void setKappa(double val) { kappa_ = val; }
        void setStateSize(int n);
        void setObservationSize(int m);
        void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }
        void setR(const Eigen::MatrixXd& R) { R_ = R; }
        void setStateModel(ModelFunction f) { f_model_ = f; }
        void setObservationModel(ModelFunction h) { h_model_ = h; }
        void setXhat(const Eigen::VectorXd& xhat) { xhat_ = xhat; }
        void setP(const Eigen::MatrixXd& P) { P_ = P; }

        inline Eigen::VectorXd get_xhat() { return xhat_; }
        inline Eigen::MatrixXd get_P() { return P_; }

        std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd>
        uTransform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt);

        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd y, double dt);
    };
} // namespace potbot_lib

#endif // HPP_POTBOT_LIB_UNSCENTED_KALMAN_FILTER_
