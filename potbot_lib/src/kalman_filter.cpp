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
        z.resize(4,1);
        xhat<<	0,
                0,
                0,
                0;
        xtilde = z = xhat;

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

    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> KalmanFilter::update(Eigen::MatrixXd data, double dt)
    {
        z = data;
        An(0,2) = dt;
        An(1,3) = dt;


        // std::cout<<I<<std::endl;
        Ptilde = An*Phat*An.transpose() + model_sigma;
        
        K = Ptilde*(Ptilde+obse_sigma).inverse();
        xhat = xtilde + K*(z - xtilde);
        Phat = (I - K)*Ptilde;

        xtilde = An*xhat;

        Eigen::VectorXd vec(4);
        vec(0) = xhat(0);
        vec(1) = xhat(1);
        vec(2) = xhat(2);
        vec(3) = xhat(3);
        return std::make_tuple(xhat, Phat, K);
    }
}