#include <potbot_lib/kalman_filter.h>

namespace potbot_lib{

    KalmanFilter::KalmanFilter()
    {
        An.resize(4,4);
        An<<1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;

        xhat.resize(4,1);
        xtilde.resize(4,1);
        ytilde.resize(4,1);
        z.resize(4,1);
        xhat<<	0,
                0,
                0,
                0;
        xtilde = xhat;
        ytilde = z;

        Phat.resize(4,4);
        Phat<<	10000,0,0,0,
                0,10000,0,0,
                0,0,10000,0,
                0,0,0,10000;

        obse_sigma.resize(4,4);
        model_sigma.resize(4,4);
        obse_sigma<<	0.01,0,0,0,
                        0,0.01,0,0,
                        0,0,0.01,0,
                        0,0,0,0.01;
        model_sigma = obse_sigma;

        K.resize(4,4);
        K<<	0,0,0,0,
            0,0,0,0,
            0,0,0,0,
            0,0,0,0;
        I = Eigen::MatrixXd::Identity(4,4);
    };

    void KalmanFilter::setA(Eigen::MatrixXd mat)
    {
        An.resize(mat.rows(), mat.cols());
        An = mat;
        int r = An.rows();
        int c = An.cols();

        xhat.resize(c,1);
        xhat.setZero();
        xtilde.resize(c,1);
        xtilde.setZero();

        Phat.resize(c,c);
        Phat.setZero();
        for (int i = 0; i < c; i++)
        {
            Phat(i,i) = 10000;
        }

        model_sigma.resize(c,c);
        model_sigma.setZero();
        for (int i = 0; i < c; i++)
        {
            model_sigma(i,i) = 0.01;
        }

        K.resize(c,c);
        K.setZero(c,c);
    }

    void KalmanFilter::setC(Eigen::MatrixXd mat)
    {
        C.resize(mat.rows(), mat.cols());
        C = mat;
        int r = C.rows();
        int c = C.cols();

        z.resize(r,1);
        z.setZero();
        ytilde.resize(r,1);
        ytilde.setZero();

        obse_sigma.resize(r,r);
        obse_sigma.setZero();
        for (int i = 0; i < r; i++)
        {
            obse_sigma(i,i) = 0.01;
        }
    }

    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> KalmanFilter::update(Eigen::MatrixXd data, double dt)
    {
        I = Eigen::MatrixXd::Identity(K.rows(),C.cols());

        // z = data;
        // An(0,2) = dt;
        // An(1,3) = dt;

        // Ptilde = An*Phat*An.transpose() + model_sigma;
        
        // K = Ptilde*(Ptilde+obse_sigma).inverse();
        // xhat = xtilde + K*(z - xtilde);
        // Phat = (I - K)*Ptilde;

        // xtilde = An*xhat;

        // Eigen::VectorXd vec(4);
        // vec(0) = xhat(0);
        // vec(1) = xhat(1);
        // vec(2) = xhat(2);
        // vec(3) = xhat(3);Ptilde
        // return std::make_tuple(xhat, Phat, K);

        //ekf
        z = data;

        Ptilde = An*Phat*An.transpose() + model_sigma;
        
        K = Ptilde*C.transpose()*(C*Ptilde*C.transpose()+obse_sigma).inverse();
        xhat = xtilde + K*(z - ytilde);
        Phat = (I - K*C)*Ptilde;

        xtilde = An*xhat;
        ytilde = C*xhat;

        return std::make_tuple(xhat, Phat, K);
    }
}