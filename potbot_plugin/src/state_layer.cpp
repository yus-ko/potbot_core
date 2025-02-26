#include <potbot_plugin/state_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::StateLayer, costmap_2d::Layer)

#define __NX__ 5
#define __NY__ 2

namespace potbot_nav
{
    void KalmanFilterROS::set_state_estimator(int value)
    {
        state_estimator_=value;
        ukf_id_.clear();
        states_kf_.clear();
        states_ukf_.clear();
    }
    

    Eigen::VectorXd f(Eigen::VectorXd x_old, double dt) {
        Eigen::VectorXd x_new(__NX__);

        x_new(0) = x_old(0) + x_old(3)*cos(x_old(2))*dt;
        x_new(1) = x_old(1) + x_old(3)*sin(x_old(2))*dt;
        x_new(2) = x_old(2) + x_old(4)*dt;
        x_new(3) = x_old(3);
        x_new(4) = x_old(4);

        return x_new;
    }

    Eigen::VectorXd h(Eigen::VectorXd x, double dt) {
        Eigen::VectorXd y(__NY__);
        y(0) = x(0);
        y(1) = x(1);
        return y;
    }
    
    void KalmanFilterROS::update(potbot_msgs::ObstacleArray& obstacles)
    {
        double t_now = obstacles.header.stamp.toSec();
        if (time_pre_ > 0 && !obstacles.data.empty())
        {
            double dt = t_now-time_pre_;

            potbot_msgs::StateArray state_array_msg;
            for(auto& obstacle : obstacles.data)
            {
                if (!obstacle.is_moving) continue;
                
                double x = obstacle.pose.position.x;
                double y = obstacle.pose.position.y;
                int id = obstacle.id;

                auto iter = std::find(ukf_id_.begin(), ukf_id_.end(), id);
                if (iter != ukf_id_.end())
                {
                    if (state_estimator_ == KALMAN_FILTER)
                    {
                        int index_ukf = std::distance(ukf_id_.begin(), iter);
                        Eigen::MatrixXd od(4,1);
                        od<< x, y,0,0;
                        ROS_INFO_STREAM(od.transpose());
                        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_kf_[index_ukf].update(od,dt);
                        Eigen::VectorXd xhat = std::get<0>(ans);
                        Eigen::MatrixXd P = std::get<1>(ans);
                        Eigen::MatrixXd K = std::get<2>(ans);

                        potbot_msgs::State state_msg;
                        state_msg.header = obstacle.header;
                        state_msg.id = id;

                        state_msg.state.resize(4);

                        state_msg.state[0].label = "z";
                        state_msg.state[0].matrix = potbot_lib::utility::matrix_to_multiarray(od);

                        state_msg.state[1].label = "xhat";
                        state_msg.state[1].matrix = potbot_lib::utility::matrix_to_multiarray(xhat);

                        state_msg.state[2].label = "P";
                        state_msg.state[2].matrix = potbot_lib::utility::matrix_to_multiarray(P);

                        state_msg.state[3].label = "K";
                        state_msg.state[3].matrix = potbot_lib::utility::matrix_to_multiarray(K);
                        
                        //std::cout<<xhat.transpose()<<std::endl;
                        state_array_msg.data.push_back(state_msg);

                        obstacle.pose.position.x = xhat(0);
                        obstacle.pose.position.y = xhat(1);
                        obstacle.twist.linear.x = xhat(2);
                        obstacle.twist.linear.y = xhat(3);
                    }
                    else if (state_estimator_ == EXTENDED_KALMAN_FILTER)
                    {
                        double v = obstacle.twist.linear.x;
                        double theta = tf2::getYaw(obstacle.pose.orientation);
                        Eigen::MatrixXd A(5,5);
                        A<< 1, 0, -v*sin(theta)*dt, cos(theta)*dt, 0,
                            0, 1, v*cos(theta)*dt, sin(theta)*dt, 0,
                            0,0,1,0,dt,
                            0,0,0,1,0,
                            0,0,0,0,1;
                        // ROS_INFO_STREAM("\n"<<A);

                        Eigen::MatrixXd C(2,5);
                        C<< 1,0,0,0,0,
                            0,1,0,0,0;

                        int index_ukf = std::distance(ukf_id_.begin(), iter);
                        Eigen::MatrixXd observed_data(2,1);
                        observed_data<< x, y;
                        // ROS_INFO("x: %.2f, y: %.2f", x,y);
                        states_kf_[index_ukf].setA(A);
                        states_kf_[index_ukf].setC(C);
                        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_kf_[index_ukf].update(observed_data,dt);
                        Eigen::VectorXd xhat = std::get<0>(ans);
                        Eigen::MatrixXd P = std::get<1>(ans);
                        Eigen::MatrixXd K = std::get<2>(ans);

                        potbot_msgs::State state_msg;
                        state_msg.header = obstacle.header;
                        state_msg.id = id;

                        state_msg.state.resize(4);

                        state_msg.state[0].label = "z";
                        state_msg.state[0].matrix = potbot_lib::utility::matrix_to_multiarray(observed_data);

                        state_msg.state[1].label = "xhat";
                        state_msg.state[1].matrix = potbot_lib::utility::matrix_to_multiarray(xhat);

                        state_msg.state[2].label = "P";
                        state_msg.state[2].matrix = potbot_lib::utility::matrix_to_multiarray(P);

                        state_msg.state[3].label = "K";
                        state_msg.state[3].matrix = potbot_lib::utility::matrix_to_multiarray(K);
                        
                        // std::cout<<xhat.transpose()<<std::endl;
                        state_array_msg.data.push_back(state_msg);

                        obstacle.pose.position.x = xhat(0);
                        obstacle.pose.position.y = xhat(1);
                        obstacle.pose.orientation  = potbot_lib::utility::get_quat(0,0,xhat(2));
                        // ROS_INFO("estimated: id:%d, th:%3f,%3f, dt:%f", id, theta, tf2::getYaw(obstacle.pose.orientation), dt);
                        obstacle.twist.linear.x = xhat(3);
                        obstacle.twist.angular.z = xhat(4);
                    }
                    else if (state_estimator_ == UNSCENTED_KALMAN_FILTER)
                    {
                        int index_ukf = std::distance(ukf_id_.begin(), iter);
                        Eigen::VectorXd observed_data(__NY__);
                        observed_data<< x, y;
                        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = states_ukf_[index_ukf].update(observed_data,dt);
                        Eigen::VectorXd xhat = std::get<0>(ans);
                        Eigen::MatrixXd P = std::get<1>(ans);
                        Eigen::MatrixXd K = std::get<2>(ans);

                        potbot_msgs::State state_msg;
                        state_msg.header = obstacle.header;
                        state_msg.id = id;

                        state_msg.state.resize(4);

                        state_msg.state[0].label = "z";
                        state_msg.state[0].matrix = potbot_lib::utility::matrix_to_multiarray(observed_data);

                        state_msg.state[1].label = "xhat";
                        state_msg.state[1].matrix = potbot_lib::utility::matrix_to_multiarray(xhat);

                        state_msg.state[2].label = "P";
                        state_msg.state[2].matrix = potbot_lib::utility::matrix_to_multiarray(P);

                        state_msg.state[3].label = "K";
                        state_msg.state[3].matrix = potbot_lib::utility::matrix_to_multiarray(K);
                        
                        state_array_msg.data.push_back(state_msg);

                        obstacle.pose.position.x = xhat(0);
                        obstacle.pose.position.y = xhat(1);
                        obstacle.pose.orientation  = potbot_lib::utility::get_quat(0,0,xhat(2));
                        obstacle.twist.linear.x = xhat(3);
                        obstacle.twist.angular.z = xhat(4);
                    }
                    
                }
                else
                {
                    
                    Eigen::MatrixXd Q(__NY__,__NY__), R(__NX__,__NX__), P(__NX__,__NX__);
                    Q.setZero();R.setZero();P.setZero();
                    // for (int i = 0; i < __NY__; i++) Q(i,i) = sigma_q_;
                    // for (int i = 0; i < __NX__; i++) R(i,i) = sigma_r_;
                    // for (int i = 0; i < __NX__; i++) P(i,i) = sigma_p_;

                    Q<< sigma_q_, 0,
                        0, sigma_q_;
                    
                    R<< 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, sigma_r_, 0,
                        0, 0, 0, 0, sigma_r_;

                    P<< sigma_p_, 0, 0, 0, 0,
                        0, sigma_p_, 0, 0, 0,
                        0, 0, sigma_p_, 0, 0,
                        0, 0, 0, sigma_p_, 0,
                        0, 0, 0, 0, sigma_p_;

                    

                    if (state_estimator_ == KALMAN_FILTER || state_estimator_ == EXTENDED_KALMAN_FILTER)
                    {
                        potbot_lib::KalmanFilter estimate;
                        Eigen::MatrixXd A(5,5), C(2,5);
                        A.setZero(); C.setZero();
                        estimate.setA(A); estimate.setC(C);
                        estimate.initialize();
                        states_kf_.push_back(estimate);
                    }
                    else if (state_estimator_ == UNSCENTED_KALMAN_FILTER)
                    {
                        Eigen::VectorXd xhat(__NX__);
                        // xhat.setZero();
                        xhat<< x,y,0,0,0;
                        potbot_lib::UnscentedKalmanFilter estimate(f,h,R,Q,P,xhat);
                        estimate.setKappa(kappa_);
                        states_ukf_.push_back(estimate);
                    }
                    
                    ukf_id_.push_back(id);
                }

            }
        }
        time_pre_ = t_now;
    }
    
    void StateLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;

        global_frame_ = layered_costmap_->getGlobalFrameID();
        double transform_tolerance;
        nh.param("transform_tolerance", transform_tolerance, 0.2);

        std::string topics_string;
        // get the topics that we'll subscribe to from the parameter server
        nh.param("observation_sources", topics_string, std::string(""));
        ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

        // now we need to split the topics based on whitespace which we can use a stringstream for
        std::stringstream ss(topics_string);

        std::string source;
        while (ss >> source)
        {
            ros::NodeHandle source_node(nh, source);

            // get the parameters for the specific topic
            double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
            std::string topic, sensor_frame, data_type;
            bool inf_is_valid, clearing, marking;

            source_node.param("topic", topic, source);
            source_node.param("sensor_frame", sensor_frame, std::string(""));
            source_node.param("observation_persistence", observation_keep_time, 0.0);
            source_node.param("expected_update_rate", expected_update_rate, 0.0);
            source_node.param("data_type", data_type, std::string("PointCloud"));
            source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
            source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
            source_node.param("inf_is_valid", inf_is_valid, false);
            source_node.param("clearing", clearing, false);
            source_node.param("marking", marking, true);

            std::string raytrace_range_param_name, obstacle_range_param_name;

            // get the obstacle range for the sensor
            double obstacle_range = 2.5;
            if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
            {
                source_node.getParam(obstacle_range_param_name, obstacle_range);
            }

            // get the raytrace range for the sensor
            double raytrace_range = 3.0;
            if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
            {
                source_node.getParam(raytrace_range_param_name, raytrace_range);
            }

            ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
                      sensor_frame.c_str());

            // create a callback for the topic
            if (data_type == "LaserScan")
            {
                sub_scan_ = nh.subscribe(topic,1, &StateLayer::laserScanCallback, this);

            }
            else if (data_type == "Image")
            {
                std::string topic_depth, topic_info;
                source_node.param("topic_depth", topic_depth, std::string(""));
                source_node.param("topic_info", topic_info, std::string(""));

                sub_rgb_.subscribe(nh, topic, 1);
                sub_depth_.subscribe(nh, topic_depth, 1);
                sub_info_.subscribe(nh, topic_info, 1);

                sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_rgb_, sub_depth_, sub_info_));
                sync_->registerCallback(boost::bind(&StateLayer::imageCallback, this, _1, _2, _3));
            }
        }
        
        dsrv_ = NULL;
        setupDynamicReconfigure(nh);

        pub_state_marker_			    = nh.advertise<visualization_msgs::MarkerArray>(	"state/marker", 1);
	    pub_obstacles_scan_estimate_	= nh.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/estimate", 1);
        pub_scan_range_                 = nh.advertise<geometry_msgs::PolygonStamped>(	    "scan_range", 1);
        pub_scan_clustering_            = nh.advertise<visualization_msgs::MarkerArray>(	"obstacle/scan/clustering", 1);
        pub_pcl_clustering_             = nh.advertise<visualization_msgs::MarkerArray>(	"obstacle/pcl/clustering", 1);
        pub_camera_points_              = nh.advertise<visualization_msgs::MarkerArray>(	"debug/camera/points", 1);
        pub_camera_image_               = nh.advertise<sensor_msgs::Image>(                 "debug/camera/image", 1);

    }

    void StateLayer::setupDynamicReconfigure(ros::NodeHandle &nh)
    {
        dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::StatePluginConfig>(nh);
        dynamic_reconfigure::Server<potbot_plugin::StatePluginConfig>::CallbackType cb = boost::bind(
            &StateLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    StateLayer::~StateLayer()
    {
        if (dsrv_)
            delete dsrv_;
    }

    void StateLayer::reconfigureCB(potbot_plugin::StatePluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
        apply_cluster_to_localmap_ = config.apply_localmap_threshold_2d_size;
        max_estimated_linear_velocity_ = config.max_estimated_linear_velocity;
        max_estimated_angular_velocity_ = config.max_estimated_angular_velocity;
        prediction_time_ = config.prediction_time;

        static std::string estimator_name = config.state_estimator;
        int state_estimator = UNSCENTED_KALMAN_FILTER;
        if (config.state_estimator != estimator_name)
        {
            if (estimator_name == "Kalman Filter")
            {
                state_estimator = KALMAN_FILTER;
            }
            else if (estimator_name == "Extended Kalman Filter")
            {
                state_estimator = EXTENDED_KALMAN_FILTER;
            }
            else if (estimator_name == "Unscented Kalman Filter")
            {
                state_estimator = UNSCENTED_KALMAN_FILTER;
            }
            kf_scan_.set_state_estimator(state_estimator);
            kf_pcl_.set_state_estimator(state_estimator);
            kf_camera_.set_state_estimator(state_estimator);
        }
        estimator_name = config.state_estimator;

        kf_scan_.set_ukf_scaling(config.kappa);
        kf_scan_.set_covariances(config.sigma_q, config.sigma_r, config.sigma_p);
        kf_pcl_.set_ukf_scaling(config.kappa);
        kf_pcl_.set_covariances(config.sigma_q, config.sigma_r, config.sigma_p);
        kf_camera_.set_ukf_scaling(config.kappa);
        kf_camera_.set_covariances(config.sigma_q, config.sigma_r, config.sigma_p);

        euclidean_cluster_tolerance_ = config.euclidean_cluster_tolerance;
        euclidean_min_cluster_size_ = config.euclidean_min_cluster_size;
    }

    int getId(std::string str)
    {
        std::regex re("\\d+"); // 数字を表す正規表現
        std::smatch match;
        std::vector<int> numbers;

        // 数字をすべて抽出
        while (std::regex_search(str, match, re)) {
            numbers.push_back(std::stoi(match.str()));
            str = match.suffix(); // 次の検索のために文字列を更新
        }

        return numbers.front();
    }

    void StateLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr &message)
    {   
        // ROS_INFO("laserScanCallback");

        sensor_msgs::LaserScan scan_data = *message;

        geometry_msgs::PolygonStamped scan_range;
        scan_range.header = scan_data.header;
        for (int i = 0; i < scan_data.ranges.size(); i+=3)
        {
            // double r = scan_data.ranges[i];
            // if (r > scan_data.range_max) r = scan_data.range_max;
            double r = scan_data.range_max;

            double th = i*scan_data.angle_increment + scan_data.angle_min;
            geometry_msgs::Point32 p;
            p.x = r*cos(th);
            p.y = r*sin(th);
            scan_range.polygon.points.push_back(p);
        }
        pub_scan_range_.publish(scan_range);

        // __MedianFilter(scan_);
        // pub_scan_filter_.publish(scan_data);
        // std::vector<SEGMENT> segments;

        potbot_lib::ScanClustering scanclus;
        scanclus.setClusters(scan_data);   //センサーデータの登録
        scanclus.euclideanClustering();    //ユークリッド距離に基づいたクラスタリングを実行

        potbot_msgs::ObstacleArray clusters_obstaclearray_scan;
        clusters_obstaclearray_scan.header = scan_data.header;
        scanclus.toObstaclearray(clusters_obstaclearray_scan);    //クラスタリング結果をpotbot_msgs::ObstacleArray型に変換して取得
        // for (auto& obs : clusters_obstaclearray_scan.data) obs.header = clusters_obstaclearray_scan.header;

        //1時刻前のクラスタからの追跡
        static potbot_msgs::ObstacleArray clusters_obstaclearray_pre;
        potbot_lib::utility::associate_obstacle(clusters_obstaclearray_scan, clusters_obstaclearray_pre, *tf_);

        scanclus.setClusters(clusters_obstaclearray_scan);

        //クラスタをワールド座標系に変換して1時刻先の追跡用データにする
        potbot_lib::utility::get_tf(*tf_, clusters_obstaclearray_scan, global_frame_, clusters_obstaclearray_pre);

        visualization_msgs::MarkerArray clusters_markerarray;
        scanclus.toMarkerarray(clusters_markerarray);  //クラスタリング結果をvisualization_msgs::MarkerArray型に変換して取得
        for (auto& clus : clusters_markerarray.markers) clus.header = scan_data.header;
        pub_scan_clustering_.publish(clusters_markerarray);

        potbot_msgs::ObstacleArray obstacle_array = clusters_obstaclearray_pre;
        // estimateState(obstacle_array);
        kf_scan_.update(obstacle_array);
        applyCloud(obstacle_array);
    }

    void StateLayer::imageCallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
    {
        // ROS_INFO("imageCallback");
        std::string camera_image_frame = rgb_msg->header.frame_id;
        ros::Time time_now = ros::Time::now();
        static ros::Time time_pre = time_now;

        //人物検出手法
        cv::HOGDescriptor hog;
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

        cv_bridge::CvImagePtr bridge_image_rgb, bridge_image_depth;
		
		try
		{
			bridge_image_rgb=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e)
		{
			std::cout<<"depth_image_callback Error \n";
			ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
			return;
		}

        try
		{
			bridge_image_depth=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch(cv_bridge::Exception& e)
		{
			std::cout<<"depth_image_callback Error \n";
			ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
			return;
		}

		cv::Mat image_rgb = bridge_image_rgb->image.clone();
        cv::Mat image_depth = bridge_image_depth->image.clone();

		cv::Mat distCoeffs = cv::Mat(info_msg->D.size(), 1, CV_64F);
		for (size_t i = 0; i < info_msg->D.size(); ++i) {
			distCoeffs.at<double>(i, 0) = info_msg->D[i];
		}

		// カメラ行列 K (3x3行列)
		cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, (void*)info_msg->K.data()).clone();

		// 回転行列 R (3x3行列)
		cv::Mat R = cv::Mat(3, 3, CV_64F, (void*)info_msg->R.data()).clone();

		// 投影行列 P (3x4行列)
		cv::Mat projectionMatrix = cv::Mat(3, 4, CV_64F, (void*)info_msg->P.data()).clone();

		double cx = cameraMatrix.at<double>(0,2);
		double cy = cameraMatrix.at<double>(1,2);
		double fx = cameraMatrix.at<double>(0,0);
		double fy = cameraMatrix.at<double>(1,1);
		double tx = projectionMatrix.at<double>(0,3);
		double ty = projectionMatrix.at<double>(1,3);
		double tz = projectionMatrix.at<double>(2,3);
        
        std::vector<cv::Rect> detections;
        hog.detectMultiScale(image_rgb, detections);
        
        visualization_msgs::MarkerArray depth_points_msgs;
        potbot_msgs::ObstacleArray obstacle_array;
        potbot_msgs::Obstacle obstacle;
        obstacle.header.frame_id = camera_image_frame;
        obstacle.header.stamp = time_now;
        obstacle.pose = potbot_lib::utility::get_pose(0, 0, 0);
        // obstacle.id = obstacle_array.data.size();
        obstacle.points.push_back(obstacle.pose.position);
        // obstacle_array.data.push_back(obstacle);
        for (const auto& rect : detections) {
            cv::rectangle(image_rgb, rect, cv::Scalar(0, 255, 0), 2);
            
            int px = rect.x + rect.width/2;
            int py = rect.y + rect.height/2;
            float depth = image_depth.at<float>(cv::Point(px,py));

            double x = (px - cx) / fx;
            double y = (py - cy) / fy;

            double width = rect.width / fx;
            double height = rect.height / fy;

            potbot_lib::Point depth_point(depth*x, depth*y, depth);

            potbot_msgs::Obstacle obstacle;
            obstacle.header.frame_id = camera_image_frame;
            obstacle.header.stamp = time_now;
            obstacle.pose = potbot_lib::utility::get_pose(depth_point.x, depth_point.y, depth_point.z);
            obstacle.scale.x = 0.05;
            obstacle.scale.y = 0.05;
            obstacle.scale.z = 0.05;
            // obstacle.id = obstacle_array.data.size();
            // obstacle.points.push_back(obstacle.pose.position);
            obstacle.is_moving = true;

            if (debug_)
            {
                potbot_lib::Point mean_point(0,0,0);
                for (int py = rect.y; py < rect.y + rect.height; py++)
                {
                    for (int px = rect.x; px < rect.x + rect.width; px++)
                    {
                        float depth = image_depth.at<float>(cv::Point(px,py));
                        double x = (px - cx) / fx;
                        double y = (py - cy) / fy;
                        if (isfinite(x) && isfinite(y) && isfinite(depth))
                        {
                            potbot_lib::Point depth_point(depth*x, depth*y, depth);
                            mean_point=mean_point+depth_point;
                            obstacle.points.push_back(potbot_lib::utility::get_point(depth_point));
                        }
                    }
                }

                mean_point=mean_point/double(obstacle.points.size());
                // obstacle.pose = potbot_lib::utility::get_pose(mean_point.x, mean_point.y, mean_point.z);
            }
            obstacle_array.data.push_back(obstacle);
        }
        // pub_camera_points_.publish(depth_points_msgs);

        std_msgs::Header header = rgb_msg->header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage cv_img_dst(header, "bgr8", image_rgb);
        pub_camera_image_.publish(cv_img_dst.toImageMsg());

        // cv::imshow("Person Detection", image_rgb);
        // cv::waitKey(1);
        if (!obstacle_array.data.empty())
        {
            obstacle_array.header = obstacle_array.data.front().header;
            potbot_msgs::ObstacleArray obstacle_array_world;
            potbot_lib::utility::get_tf(*tf_, obstacle_array, global_frame_, obstacle_array_world);
            static potbot_msgs::ObstacleArray obstaclearray_pre = obstacle_array_world;
            potbot_lib::utility::associate_obstacle(obstacle_array_world, obstaclearray_pre, *tf_);
            kf_camera_.update(obstacle_array_world);
            applyCloud(obstacle_array_world);
            obstaclearray_pre = obstacle_array_world;
            visualization_msgs::MarkerArray obstacle_marker_array;
            potbot_lib::utility::to_msg(
                obstacle_array_world, obstacle_marker_array,
                (time_now-time_pre).toSec()-0.01,
                visualization_msgs::Marker::CUBE,
                visualization_msgs::Marker::ADD
            );
            pub_camera_points_.publish(obstacle_marker_array);
        }
        
        time_pre = time_now;
    }

    void StateLayer::applyCloud(const potbot_msgs::ObstacleArray& obstacles)
    {
        double time_now = obstacles.header.stamp.toSec();
        static double time_pre = time_now;
        double dt=time_now-time_pre;
        if (!obstacles.data.empty() && dt>0)
        {
            scan_cloud_.clear();
            // pcl::fromROSMsg(cloud, cloudxyz);

            visualization_msgs::MarkerArray state_markers;
            potbot_lib::utility::obstacle_array_to_marker_array(obstacles, state_markers);
            pub_state_marker_.publish(state_markers);
            pub_obstacles_scan_estimate_.publish(obstacles);

            for (const auto& obs:obstacles.data)
            {

                potbot_msgs::Obstacle wobs;
                potbot_lib::utility::get_tf(*tf_, obs, global_frame_, wobs);
                for (const auto& p:wobs.points)
                {
                    scan_cloud_.push_back(pcl::PointXYZ(p.x,p.y,p.z));
                }

                if (!obs.is_moving) continue;

                double omega                = fmod(wobs.twist.angular.z, 2*M_PI); //障害物の回転角速度
                double yaw                  = tf2::getYaw(wobs.pose.orientation);  //障害物の姿勢
                double width                = wobs.scale.y; //障害物の幅
                double depth                = wobs.scale.x; //障害物の奥行き
                double size                 = width + depth;

                double vx                   = wobs.twist.linear.x;  //障害物の並進速度
                double vy                   = wobs.twist.linear.y;  //障害物の並進速度

                ROS_DEBUG("estimated: id:%d, x:%f, y:%f, th:%f, vx:%f, vy:%f, w:%f, dt:%f, size:%f/%f", obs.id, wobs.pose.position.x, wobs.pose.position.y, yaw, vx, vy, omega, dt, size, apply_cluster_to_localmap_);
                if (size < apply_cluster_to_localmap_)
                {
                    if (abs(omega) < max_estimated_angular_velocity_ && abs(vx) < max_estimated_linear_velocity_ && abs(vy) < max_estimated_linear_velocity_)
                    {
                        //並進速度と角速度を一定として1秒後までの位置x,yを算出
                        for (double t = 0; t < prediction_time_; t += dt)
                        {
                            double angle = omega*t + yaw;
                            double dx = vx*t;
                            double dy = vy*t;

                            for (const auto& p : wobs.points)
                            {
                                double x            = dx*cos(angle) - dy*sin(angle) + p.x;
                                double y            = dx*sin(angle) + dy*cos(angle) + p.y;
                                scan_cloud_.push_back(pcl::PointXYZ(x,y,p.z));
                            }
                        }
                    }
                }
            }
        }
        time_pre = time_now;
    }

    void StateLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
            return;

        *min_x = std::min(*min_x, -10.0);
        *min_y = std::min(*min_y, -10.0);
        *max_x = std::max(*max_x, 10.0);
        *max_y = std::max(*max_y, 10.0);

        // ROS_INFO("%f, %f, %f, %f", *min_x, *min_y, *max_x, *max_y);
    }

    void StateLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;

        // if (footprint_clearing_enabled_)
        // {
        //     setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
        // }

        current_ = true;
        for (const auto& p:scan_cloud_)
        {
            unsigned int mx,my;
            if (master_grid.worldToMap(p.x,p.y,mx,my))
            {
                master_grid.setCost(mx,my,LETHAL_OBSTACLE);
            }
        }
        
    }

} // namespace potbot_nav
