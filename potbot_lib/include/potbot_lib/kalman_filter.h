#ifndef H_KALMANFILTER_
#define H_KALMANFILTER_

#include <potbot_lib/utility_ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potbot_lib
{
	class KalmanFilter{
		private:

			Eigen::VectorXd xtilde, ytilde;
			Eigen::VectorXd xhat;	//推定状態
			Eigen::VectorXd z;	//観測データ
			Eigen::MatrixXd Phat, Ptilde,
			An,				//障害物の移動モデル
			C,				//センサの観測モデル
			obse_sigma,		//観測誤差共分散
			model_sigma,	//モデル化誤差共分散
			K,				//カルマンゲイン
			I;				//単位行列

			double dt = -1;		//サンプリング時間

		public:
			KalmanFilter();
			~KalmanFilter(){};
			
			//カルマンフィルタパラメーターの初期化
			void initialize();

			//更新ステップの実行
			std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> update(Eigen::VectorXd data, double dt);

			//行列セッター　観測対象のシステムモデル
			void setA(Eigen::MatrixXd mat);	

			//行列セッター　観測に使用するセンサの観測モデル
			void setC(Eigen::MatrixXd mat);

			//行列ゲッター　推定状態ベクトル
			inline Eigen::VectorXd get_xhat()
			{
				return xhat;
			};

			//行列ゲッター　観測ベクトル
			inline Eigen::VectorXd get_z()
			{
				return z;
			};

			//行列ゲッター　カルマンゲイン
			inline Eigen::MatrixXd get_K()
			{
				return K;
			};

			//行列ゲッター　推定共分散行列
			inline Eigen::MatrixXd get_P()
			{
				return Phat;
			};

	};
}
#endif // H_KALMANFILTER_