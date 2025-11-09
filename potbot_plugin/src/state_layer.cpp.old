/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <potbot_plugin/state_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::StateLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using costmap_2d::Observation;
using costmap_2d::ObservationBuffer;

#define __NX__ 5
#define __NY__ 2

namespace potbot_nav
{

    void StateLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;
        rolling_window_ = layered_costmap_->isRolling();

        bool track_unknown_space;
        nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
        if (track_unknown_space)
            default_value_ = NO_INFORMATION;
        else
            default_value_ = FREE_SPACE;

        StateLayer::matchSize();
        current_ = true;

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

            // create an observation buffer
            observation_buffers_.push_back(
                boost::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                                                           max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                                                           sensor_frame, transform_tolerance)));

            // check if we'll add this buffer to our marking observation buffers
            if (marking)
                marking_buffers_.push_back(observation_buffers_.back());

            // check if we'll also add this buffer to our clearing observation buffers
            if (clearing)
                clearing_buffers_.push_back(observation_buffers_.back());

            ROS_DEBUG(
                "Created an observation buffer for source %s, topic %s, global frame: %s, "
                "expected update rate: %.2f, observation persistence: %.2f",
                source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

            // create a callback for the topic
            if (data_type == "LaserScan")
            {
                boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

                boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
                    new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

                if (inf_is_valid)
                {
                    filter->registerCallback(boost::bind(&StateLayer::laserScanValidInfCallback, this, _1,
                                                         observation_buffers_.back()));
                }
                else
                {
                    filter->registerCallback(boost::bind(&StateLayer::laserScanCallback, this, _1, observation_buffers_.back()));
                }

                observation_subscribers_.push_back(sub);
                observation_notifiers_.push_back(filter);

                observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
            }
            else if (data_type == "PointCloud")
            {
                boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

                if (inf_is_valid)
                {
                    ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
                }

                boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud>> filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
                filter->registerCallback(
                    boost::bind(&StateLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

                observation_subscribers_.push_back(sub);
                observation_notifiers_.push_back(filter);
            }
            else
            {
                boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

                if (inf_is_valid)
                {
                    ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
                }

                boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
                filter->registerCallback(
                    boost::bind(&StateLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

                observation_subscribers_.push_back(sub);
                observation_notifiers_.push_back(filter);
            }

            if (sensor_frame != "")
            {
                std::vector<std::string> target_frames;
                target_frames.push_back(global_frame_);
                target_frames.push_back(sensor_frame);
                observation_notifiers_.back()->setTargetFrames(target_frames);
            }
        }
        
        dsrv_ = NULL;
        setupDynamicReconfigure(nh);

        pub_state_marker_			    = nh.advertise<visualization_msgs::MarkerArray>(	"state/marker", 1);
	    pub_obstacles_scan_estimate_	= nh.advertise<potbot_msgs::ObstacleArray>(			"obstacle/scan/estimate", 1);
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
        footprint_clearing_enabled_ = config.footprint_clearing_enabled;
        max_obstacle_height_ = config.max_obstacle_height;
        combination_method_ = config.combination_method;
        apply_cluster_to_localmap_ = config.apply_localmap_threshold_2d_size;
        max_estimated_linear_velocity_ = config.max_estimated_linear_velocity;
        max_estimated_angular_velocity_ = config.max_estimated_angular_velocity;
        prediction_time_ = config.prediction_time;
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

    void StateLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr &message,
                                          const boost::shared_ptr<ObservationBuffer> &buffer)
    {
        // ROS_INFO("laserScanCallback");
        // project the laser into a point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message->header;

        // project the scan into a point cloud
        try
        {
            projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
                     ex.what());
            projector_.projectLaser(*message, cloud);
        }

        sensor_msgs::LaserScan scan_data = *message;

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
                int ny = 2;
                Eigen::VectorXd observed_data(ny);
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

                states_ukf_.push_back(estimate);
                ukf_id.push_back(id);
            }

        }
        t_pre = t_now;

        if (!obstacle_array.data.empty())
        {
            pcl::PointCloud<pcl::PointXYZ> cloudxyz;
            // pcl::fromROSMsg(cloud, cloudxyz);

            visualization_msgs::MarkerArray state_markers;
            potbot_lib::utility::obstacle_array_to_marker_array(obstacle_array, state_markers);
            pub_state_marker_.publish(state_markers);
            pub_obstacles_scan_estimate_.publish(obstacle_array);

            for (const auto& obs:obstacle_array.data)
            {
                if (!obs.is_moving) continue;
                
                double v                    = obs.twist.linear.x;  //障害物の並進速度
                double omega                = fmod(obs.twist.angular.z, 2*M_PI); //障害物の回転角速度
                double yaw                  = potbot_lib::utility::get_Yaw(obs.pose.orientation);  //障害物の姿勢
                double width                = obs.scale.y; //障害物の幅
                double depth                = obs.scale.x; //障害物の奥行き
                double size                 = width + depth;

                if (size < apply_cluster_to_localmap_)
                {
                    // ROS_INFO("est vel %d: %f, %f, %f, %f, %f", obs.id, obs.pose.position.x, obs.pose.position.y, yaw, v, omega);
                    if (abs(v) < max_estimated_linear_velocity_ && abs(omega) < max_estimated_angular_velocity_)
                    {
                        //並進速度と角速度を一定として1秒後までの位置x,yを算出
                        for (double t = 0; t < prediction_time_; t += dt)
                        {
                            double distance = v*t;
                            double angle = omega*t + yaw;
                            for (const auto& p : obs.points)
                            {
                                double x            = distance*cos(angle) + p.x;
                                double y            = distance*sin(angle) + p.y;
                                cloudxyz.push_back(pcl::PointXYZ(x,y,p.z));
                            }
                            
                        }
                    }
                }

            }

            // ROS_INFO("%d, %d, %d, %d", cloudxyz.width, cloudxyz.height, cloudxyz.is_dense, cloudxyz.points.size());

            sensor_msgs::PointCloud2 msg_apply_map;
            pcl::toROSMsg(cloudxyz, msg_apply_map);
            msg_apply_map.header = message->header;

            // buffer the point cloud
            buffer->lock();
            buffer->bufferCloud(msg_apply_map);
            buffer->unlock();
            
        }
        
    }

    void StateLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr &raw_message,
                                                  const boost::shared_ptr<ObservationBuffer> &buffer)
    {
        // ROS_INFO("laserScanValidInfCallback");
        // Filter positive infinities ("Inf"s) to max_range.
        float epsilon = 0.0001; // a tenth of a millimeter
        sensor_msgs::LaserScan message = *raw_message;
        for (size_t i = 0; i < message.ranges.size(); i++)
        {
            float range = message.ranges[i];
            if (!std::isfinite(range) && range > 0)
            {
                message.ranges[i] = message.range_max - epsilon;
            }
        }

        // project the laser into a point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message.header;

        // project the scan into a point cloud
        try
        {
            projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
                     global_frame_.c_str(), ex.what());
            projector_.projectLaser(message, cloud);
        }

        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();
    }

    void StateLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr &message,
                                           const boost::shared_ptr<ObservationBuffer> &buffer)
    {
        sensor_msgs::PointCloud2 cloud2;

        if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
        {
            ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
            return;
        }

        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(cloud2);
        buffer->unlock();
    }

    void StateLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &message,
                                            const boost::shared_ptr<ObservationBuffer> &buffer)
    {
        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(*message);
        buffer->unlock();
    }

} // namespace potbot_nav
