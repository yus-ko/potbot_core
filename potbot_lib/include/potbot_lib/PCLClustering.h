#ifndef _H_PCLCLUSTERING_
#define _H_PCLCLUSTERING_

#include <potbot_lib/Utility.h>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

namespace potbot_lib
{

    class PCLClustering
    {
        private:
            float down_sampling_voxel_size_             = 0.01;

            double plane_removal_distance_threshold_    = 0.02;

            double euclidean_cluster_tolerance_         = 0.5;
            int euclidean_min_cluster_size_             = 100;

        protected:
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_;

        public:
            PCLClustering();
            ~PCLClustering(){};
            
            void set_clusters(const potbot_msgs::ObstacleArray& obstaclearray);
            void set_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, size_t index = 0);
            
            void set_down_sampling_voxel_size(double value) { down_sampling_voxel_size_ = value; };
            void set_plane_removal_distance_threshold(double value) { plane_removal_distance_threshold_ = value; };

            void set_euclidean_cluster_tolerance(double value) { euclidean_cluster_tolerance_ = value; };
            void set_euclidean_min_cluster_size(double value) { euclidean_min_cluster_size_ = value; };
            
            void get_clusters(pcl::PCLPointCloud2& pointcloud2, size_t index = 0);
            void get_clusters(sensor_msgs::PointCloud2& cloud_ros, size_t index = 0);
            void get_clusters(visualization_msgs::MarkerArray& cloud_markers);
            void get_clusters(potbot_msgs::ObstacleArray& obstaclearray);

            void down_sampling(size_t index = 0);
            void plane_removal(size_t index = 0);
            
            void euclidean_clustering(size_t index = 0);

    };

    class PCLSuperVoxel : public PCLClustering
    {
        private:
            visualization_msgs::MarkerArray marker_array_;

            float supervoxel_voxel_resolution_          = 0.008f;
            float supervoxel_seed_resolution_           = 0.1f;
            float supervoxel_color_importance_          = 0.2f;
            float supervoxel_spatial_importance_        = 0.4f;
            float supervoxel_normal_importance_         = 1.0f;

        public:
            PCLSuperVoxel() : PCLClustering::PCLClustering(){};
            ~PCLSuperVoxel(){};

            void set_supervoxel_voxel_resolution(double value) { supervoxel_voxel_resolution_ = value; };
            void set_supervoxel_seed_resolution(double value) { supervoxel_seed_resolution_ = value; };
            void set_supervoxel_color_importance(double value) { supervoxel_color_importance_ = value; };
            void set_supervoxel_spatial_importance(double value) { supervoxel_spatial_importance_ = value; };
            void set_supervoxel_normal_importance(double value) { supervoxel_normal_importance_ = value; };

            void get_clusters(pcl::PCLPointCloud2& pointcloud2, size_t index = 0) { PCLClustering::get_clusters(pointcloud2, index); };
            void get_clusters(sensor_msgs::PointCloud2& cloud_ros, size_t index = 0) { PCLClustering::get_clusters(cloud_ros, index); };
            void get_clusters(visualization_msgs::MarkerArray& cloud_markers);
            void get_clusters(potbot_msgs::ObstacleArray& obstaclearray) {};

            void supervoxel_clustering(size_t index = 0);

    };

}

#endif	// _H_PCLCLUSTERING_