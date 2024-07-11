#include <potbot_lib/unscented_kalman_filter.h>

namespace potbot_lib{

    UnscentedKalmanFilter::UnscentedKalmanFilter(ModelFunction model_func_system, ModelFunction model_func_observ, 
        Eigen::MatrixXd noise_covariance_system, Eigen::MatrixXd noise_covariance_observ, 
        Eigen::MatrixXd error_covariance_ini, Eigen::VectorXd estimate_state_ini)
    {
        f_      = model_func_system;
        h_      = model_func_observ;
        R_      = noise_covariance_system;
        Q_      = noise_covariance_observ;
        P_      = error_covariance_ini;
        xhat_   = estimate_state_ini;
    };
    UnscentedKalmanFilter::~UnscentedKalmanFilter(){};

    // U変換(unscented transform)
    std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> UnscentedKalmanFilter::uTransform(ModelFunction f_ut, Eigen::VectorXd xm, Eigen::MatrixXd Pxx, double dt) 
    {
        bool result = false;
        double n = xm.rows();
        // double kappa = 3.0-n;
        double kappa = kappa_;

        Eigen::VectorXd w(2*(int)n+1);
        w.fill(1.0/(2*(n+kappa)));
        w(0) = kappa/(n+kappa);

        //std::cout<<"Pxx =\n"<<Pxx<<"\n"<<std::endl;
        Eigen::MatrixXd L;
        Eigen::LLT<Eigen::MatrixXd> llt(Pxx);  // Pをコレスキー分解
        if (llt.info() == Eigen::Success) {
            L = llt.matrixL();  // 下三角行列Lを取得
            result = true;
        } else {
            std::cout << "Pxx =\n" << Pxx << std::endl;
            std::cout << "Matrix is not positive definite." << std::endl;
            return std::make_tuple(result, xm, Pxx, Pxx);
        }

        //シグマポイントを作成
        Eigen::MatrixXd X((int)n, 2*(int)n+1);
        X.col(0)=xm;
        X.block(0, 1,					(X.cols()-1)/2, (X.cols()-1)/2) = xm*Eigen::RowVectorXd::Ones((int)n) + sqrt(n+kappa)*L;
        X.block(0, (X.cols()-1)/2+1,	(X.cols()-1)/2, (X.cols()-1)/2) = xm*Eigen::RowVectorXd::Ones((int)n) - sqrt(n+kappa)*L;

        //シグマポイントをfで変換
        int num_dim = f_ut(X.col(0),dt).rows();
        Eigen::MatrixXd Y(num_dim, X.cols());
        for(int i=0; i<Y.cols(); i++) Y.col(i) = f_ut(X.col(i),dt);

        // 重みをかけながら行ごとの総和を計算する(yの期待値計算)
        Eigen::VectorXd ym = (Y.array().rowwise() * w.transpose().array()).rowwise().sum();

        // Eigen::VectorXd I(Y.cols());
        // I.setOnes();
        // Eigen::MatrixXd Yd(Y.rows(), Y.cols());
        // for(int i=0; i<Y.rows(); i++) Yd.row(i) = Y.row(i) - (ym(i)*I).transpose();
        
        // I.resize(X.cols());
        // I.setOnes();
        // Eigen::MatrixXd Xd(X.rows(), X.cols());
        // for(int i=0; i<X.rows(); i++) Xd.row(i) = X.row(i) - (xm(i)*I).transpose();

        // Eigen::MatrixXd Pyy = Yd*w.asDiagonal()*Yd.transpose();
        // Eigen::MatrixXd Pxy = Xd*w.asDiagonal()*Yd.transpose();

        Eigen::MatrixXd Pyy(num_dim, num_dim);
        Pyy.setZero();
        for(int i=0; i<Y.cols(); i++) Pyy += w(i)*(Y.col(i) - ym)*(Y.col(i) - ym).transpose();
        
        Eigen::MatrixXd Pxy(X.rows(), num_dim);
        Pxy.setZero();
        for(int i=0; i<Y.cols(); i++) Pxy += w(i)*(X.col(i) - xm)*(Y.col(i) - ym).transpose();
        
        // std::cout<<"Pyy =\n"<<Pyy<<std::endl;
        return std::make_tuple(result, ym, Pyy, Pxy);
    };

    std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> UnscentedKalmanFilter::update(Eigen::VectorXd y, double dt)
    {
        
        Eigen::MatrixXd G(xhat_.rows(), y.rows());
        
        std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans1 = uTransform(f_,xhat_,P_,dt);

        bool success                = std::get<0>(ans1);
        if (!success) return std::make_tuple(xhat_, P_, G);

        Eigen::VectorXd xhatm       = std::get<1>(ans1);
        Eigen::MatrixXd Pm          = std::get<2>(ans1);

        Pm                          += R_;

        std::tuple<bool, Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans2 = uTransform(h_,xhatm,Pm,dt);

        success                     = std::get<0>(ans2);
        if (!success) return std::make_tuple(xhat_, P_, G);

        Eigen::VectorXd yhatm       = std::get<1>(ans2);
        Eigen::MatrixXd Pyy         = std::get<2>(ans2);
        Eigen::MatrixXd Pxy         = std::get<3>(ans2);

        G                           = Pxy*(Pyy+Q_).inverse();			//カルマンゲイン
        Eigen::VectorXd xhat_new    = xhatm + G*(y-yhatm);              //推定値
        Eigen::MatrixXd P_new       = Pm - G*Pxy.transpose();           //推定誤差共分散

        xhat_                       = xhat_new;
        P_                          = P_new;

        // std::cout<<"P_new =\n"<<P_new<<"\n"<<std::endl;
        return std::make_tuple(xhat_new, P_new, G);
    }
}