#ifndef H_KALMANFILTER_
#define H_KALMANFILTER_

#include <potbot_lib/utility_ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potbot_lib
{
	class KalmanFilter{
		private:
			Eigen::MatrixXd Phat, Ptilde, xtilde,
			xhat,			//推定状態
			z,				//観測データ
			An,				//障害物の移動モデル
			obse_sigma,		//観測誤差共分散
			model_sigma,	//モデル化誤差共分散
			K,				//カルマンゲイン
			I;				//単位行列

			double dt = -1;		//サンプリング時間

		public:
			KalmanFilter();
			~KalmanFilter(){};

			std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::MatrixXd data, double dt);

			inline Eigen::MatrixXd get_xhat()
			{
				return xhat;
			};

			inline Eigen::MatrixXd get_z()
			{
				return z;
			};

			inline Eigen::MatrixXd get_K()
			{
				return K;
			};

			inline Eigen::MatrixXd get_P()
			{
				return Phat;
			};

	};
}
#endif // H_KALMANFILTER_