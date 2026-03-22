#ifndef HPP_POTBOT_ROS_PCL_CLUSTERING_
#define HPP_POTBOT_ROS_PCL_CLUSTERING_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace potbot_lib
{

    class PCLClustering
    {
    protected:
        double down_sampling_voxel_size_         = 0.1;
        double plane_removal_distance_threshold_ = 0.02;
        double euclidean_cluster_tolerance_      = 0.5;
        int    euclidean_min_cluster_size_       = 100;
        int    euclidean_max_cluster_size_       = 25000;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_;
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> clusters_;

    public:
        PCLClustering();
        ~PCLClustering() {}

        void set_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, size_t index = 0);

        void set_down_sampling_voxel_size(double value)          { down_sampling_voxel_size_ = value; }
        void set_plane_removal_distance_threshold(double value)  { plane_removal_distance_threshold_ = value; }
        void set_euclidean_cluster_tolerance(double value)       { euclidean_cluster_tolerance_ = value; }
        void set_euclidean_min_cluster_size(int value)           { euclidean_min_cluster_size_ = value; }
        void set_euclidean_max_cluster_size(int value)           { euclidean_max_cluster_size_ = value; }

        void get_clusters(pcl::PCLPointCloud2& pointcloud2, size_t index = 0);
        void get_clusters(sensor_msgs::msg::PointCloud2& cloud_ros, size_t index = 0);
        void get_clusters(visualization_msgs::msg::MarkerArray& cloud_markers);

        void down_sampling(size_t index = 0);
        void plane_removal(size_t index = 0);
        void euclidean_clustering(size_t index = 0);
    };

    class SupervoxelClustering : public PCLClustering
    {
    protected:
        double supervoxel_voxel_resolution_   = 0.008;
        double supervoxel_seed_resolution_    = 0.1;
        double supervoxel_color_importance_   = 0.2;
        double supervoxel_spatial_importance_ = 0.4;
        double supervoxel_normal_importance_  = 1.0;

    public:
        SupervoxelClustering() {}
        ~SupervoxelClustering() {}

        void set_supervoxel_voxel_resolution(double value)   { supervoxel_voxel_resolution_ = value; }
        void set_supervoxel_seed_resolution(double value)    { supervoxel_seed_resolution_ = value; }
        void set_supervoxel_color_importance(double value)   { supervoxel_color_importance_ = value; }
        void set_supervoxel_spatial_importance(double value) { supervoxel_spatial_importance_ = value; }
        void set_supervoxel_normal_importance(double value)  { supervoxel_normal_importance_ = value; }

        void supervoxel_clustering(size_t index = 0);
    };

} // namespace potbot_lib

#endif // HPP_POTBOT_ROS_PCL_CLUSTERING_
