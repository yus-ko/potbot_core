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
#ifndef POTBOT_NAV_STATE_LAYER_H_
#define POTBOT_NAV_STATE_LAYER_H_

#include <regex>

#include <ros/ros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/footprint.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Polygon.h>

#include <gazebo_msgs/ModelStates.h>
#include <potbot_plugin/StatePluginConfig.h>
#include <potbot_msgs/StateArray.h>
#include <potbot_lib/scan_clustering.h>
#include <potbot_lib/kalman_filter.h>
#include <potbot_lib/unscented_kalman_filter.h>
#include <potbot_lib/pcl_clustering.h>

using namespace costmap_2d;

#define KALMAN_FILTER 0
#define EXTENDED_KALMAN_FILTER 1
#define UNSCENTED_KALMAN_FILTER 2

namespace potbot_nav
{
    class KalmanFilterROS
    {
    public:
        KalmanFilterROS(){};
        ~KalmanFilterROS(){};
        void set_state_estimator(int value);
        void set_ukf_scaling(double value){kappa_=value;};
        void set_covariances(double q,double r,double p){sigma_q_=q; sigma_r_=r; sigma_p_=p;};
        void update(potbot_msgs::ObstacleArray& obstacles);
        
    private:
        std::vector<int> ukf_id_;
        std::vector<potbot_lib::KalmanFilter> states_kf_;
        std::vector<potbot_lib::UnscentedKalmanFilter> states_ukf_;

        double kappa_ = -2;
        double sigma_q_ = 0.00001;
        double sigma_r_ = 0.00001;
        double sigma_p_ = 1;
        int state_estimator_ = UNSCENTED_KALMAN_FILTER;

        double time_pre_ = -1;
    };

    class StateLayer : public costmap_2d::Layer
    {
    public:
        StateLayer()
        {
            // costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
        }

        virtual ~StateLayer();
        virtual void onInitialize();
        virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        /**
         * @brief  A callback to handle buffering LaserScan messages
         * @param message The message returned from a message notifier
         */
        void laserScanCallback(const sensor_msgs::LaserScanConstPtr &message);

        void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &message);

        void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message);
        void imageCallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg);

        void applyCloud(const potbot_msgs::ObstacleArray& obstacles);

    protected:
        virtual void setupDynamicReconfigure(ros::NodeHandle &nh);

    private:
        std::string global_frame_;

        ros::Subscriber sub_scan_, sub_model_states_, sub_pcl2_, sub_image_;
        ros::Publisher pub_scan_clustering_, pub_state_marker_, pub_obstacles_scan_estimate_, pub_scan_range_, pub_pcl_clustering_, pub_camera_image_, pub_camera_points_;
        message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
        message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;

        KalmanFilterROS kf_scan_, kf_pcl_, kf_camera_, kf_model_;

        bool debug_ = false;
        double apply_cluster_to_localmap_ = 100;
        double max_estimated_linear_velocity_ = 100;
        double max_estimated_angular_velocity_ = 100;
        double prediction_time_ = 2;

        double euclidean_cluster_tolerance_         = 0.5;
        int euclidean_min_cluster_size_             = 100;

        pcl::PointCloud<pcl::PointXYZ> scan_cloud_;

        dynamic_reconfigure::Server<potbot_plugin::StatePluginConfig> *dsrv_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
        boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

        void reconfigureCB(potbot_plugin::StatePluginConfig &config, uint32_t level);
    };

} // namespace potbot_nav

#endif // POTBOT_NAV_STATE_LAYER_H_
