#include <potbot_plugin/state_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::StateLayer, costmap_2d::Layer)

#define __NX__ 5
#define __NY__ 2

namespace potbot_nav
{

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
            ROS_INFO_STREAM(sensor_frame);
            source_node.param("observation_persistence", observation_keep_time, 0.0);
            source_node.param("expected_update_rate", expected_update_rate, 0.0);
            source_node.param("data_type", data_type, std::string("PointCloud"));
            source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
            source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
            source_node.param("inf_is_valid", inf_is_valid, false);
            source_node.param("clearing", clearing, false);
            source_node.param("marking", marking, true);

            if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
            {
                ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
                throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
            }

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

                // boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

                // boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
                //     new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

                // filter->registerCallback(boost::bind(&StateLayer::laserScanCallback, this, _1));

            }
        }
        
        dsrv_ = NULL;
        setupDynamicReconfigure(nh);

        pub_state_marker_			    = nh.advertise<visualization_msgs::MarkerArray>(	"state/marker", 1);
	    pub_obstacles_scan_estimate_	= nh.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/estimate", 1);
        pub_scan_range_                 = nh.advertise<geometry_msgs::PolygonStamped>(	    "scan_range", 1);
        pub_scan_clustering_            = nh.advertise<visualization_msgs::MarkerArray>(	"obstacle/scan/clustering", 1);

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

        kappa_ = config.kappa;
        sigma_q_ = config.sigma_q;
        sigma_r_ = config.sigma_r;
        sigma_p_ = config.sigma_p;
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

    void StateLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr &message)
    {
        // ROS_INFO("laserScanCallback");
        // project the laser into a point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message->header;

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



        potbot_msgs::ObstacleArray obstacle_array = clusters_obstaclearray_scan;
        static std::vector<int> ukf_id;
        
        if (obstacle_array.data.empty())
        {
            // pub_obstacles_scan_.publish(obstacle_array);
            return;
        }

        double t_now = obstacle_array.header.stamp.toSec();
        // double t_now = ros::Time::now().toSec();
        static double t_pre = t_now;
        double dt = t_now-t_pre;

        potbot_msgs::StateArray state_array_msg;
        for(auto& obstacle : obstacle_array.data)
        {
            if (!obstacle.is_moving) continue;
            
            double x = obstacle.pose.position.x;
            double y = obstacle.pose.position.y;
            int id = obstacle.id;

            auto iter = std::find(ukf_id.begin(), ukf_id.end(), id);
            if (iter != ukf_id.end())
            {
                int index_ukf = std::distance(ukf_id.begin(), iter);
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
                
                //std::cout<<xhat.transpose()<<std::endl;
                state_array_msg.data.push_back(state_msg);

                obstacle.pose.position.x = xhat(0);
                obstacle.pose.position.y = xhat(1);
                obstacle.pose.orientation  = potbot_lib::utility::get_Quat(0,0,xhat(2));
                obstacle.twist.linear.x = xhat(3);
                obstacle.twist.angular.z = xhat(4);
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

                Eigen::VectorXd xhat(__NX__);
                // xhat.setZero();
                xhat<< x,y,0,0,0;
                potbot_lib::UnscentedKalmanFilter estimate(f,h,R,Q,P,xhat);
                estimate.setKappa(kappa_);

                states_ukf_.push_back(estimate);
                ukf_id.push_back(id);
            }

        }
        t_pre = t_now;

        if (!obstacle_array.data.empty())
        {
            scan_cloud_.clear();
            // pcl::fromROSMsg(cloud, cloudxyz);

            visualization_msgs::MarkerArray state_markers;
            potbot_lib::utility::obstacle_array_to_marker_array(obstacle_array, state_markers);
            pub_state_marker_.publish(state_markers);
            pub_obstacles_scan_estimate_.publish(obstacle_array);

            for (const auto& obs:obstacle_array.data)
            {
                if (!obs.is_moving) continue;

                potbot_msgs::Obstacle wobs;
                potbot_lib::utility::get_tf(*tf_, obs, global_frame_, wobs);
                
                double v                    = wobs.twist.linear.x;  //障害物の並進速度
                double omega                = fmod(wobs.twist.angular.z, 2*M_PI); //障害物の回転角速度
                double yaw                  = potbot_lib::utility::get_Yaw(wobs.pose.orientation);  //障害物の姿勢
                double width                = wobs.scale.y; //障害物の幅
                double depth                = wobs.scale.x; //障害物の奥行き
                double size                 = width + depth;

                ROS_DEBUG("estimated: id:%d, x:%f, y:%f, th:%f, v:%f, w:%f, dt:%f, size:%f/%f", obs.id, wobs.pose.position.x, wobs.pose.position.y, yaw, v, omega, dt, size, apply_cluster_to_localmap_);
                if (size < apply_cluster_to_localmap_)
                {
                    if (abs(v) < max_estimated_linear_velocity_ && abs(omega) < max_estimated_angular_velocity_)
                    {
                        //並進速度と角速度を一定として1秒後までの位置x,yを算出
                        for (double t = 0; t < prediction_time_; t += dt)
                        {
                            double distance = v*t;
                            double angle = omega*t + yaw;
                            for (const auto& p : wobs.points)
                            {
                                double x            = distance*cos(angle) + p.x;
                                double y            = distance*sin(angle) + p.y;
                                scan_cloud_.push_back(pcl::PointXYZ(x,y,p.z));
                            }
                        }
                    }
                }
            }
        }
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

        // for (double x = 1; x<= 2; x+=0.05)
        // {
        //     for (double y = 0.8; y<= 1.2; y+=0.05)
        //     {
        //         unsigned int mx,my;
        //         if (master_grid.worldToMap(x,y,mx,my))
        //         {
        //             master_grid.setCost(mx,my,FREE_SPACE);
        //         }
        //     }
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
