#include <potbot_ros/pcl_clustering.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/supervoxel_clustering.h>

namespace potbot_lib
{

    PCLClustering::PCLClustering()
    {
    }

    void PCLClustering::set_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, size_t index)
    {
        if (index >= point_cloud_.size())
        {
            point_cloud_.resize(index + 1);
            clusters_.resize(index + 1);
        }
        point_cloud_[index] = point_cloud;
        clusters_[index].clear();
        clusters_[index].push_back(point_cloud);
    }

    void PCLClustering::down_sampling(size_t index)
    {
        if (index >= clusters_.size() || clusters_[index].empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
        for (auto& cloud : clusters_[index])
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(
                static_cast<float>(down_sampling_voxel_size_),
                static_cast<float>(down_sampling_voxel_size_),
                static_cast<float>(down_sampling_voxel_size_));
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            vg.filter(*filtered);
            result.push_back(filtered);
        }
        clusters_[index] = result;
    }

    void PCLClustering::plane_removal(size_t index)
    {
        if (index >= clusters_.size() || clusters_[index].empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
        for (auto& cloud : clusters_[index])
        {
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(plane_removal_distance_threshold_);
            seg.setInputCloud(cloud);

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty())
            {
                result.push_back(cloud);
                continue;
            }

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*filtered);
            result.push_back(filtered);
        }
        clusters_[index] = result;
    }

    void PCLClustering::euclidean_clustering(size_t index)
    {
        if (index >= clusters_.size() || clusters_[index].empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
        for (auto& cloud : clusters_[index])
        {
            if (cloud->empty()) continue;

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(euclidean_cluster_tolerance_);
            ec.setMinClusterSize(euclidean_min_cluster_size_);
            ec.setMaxClusterSize(euclidean_max_cluster_size_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            for (const auto& indices : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx : indices.indices)
                {
                    cluster->push_back((*cloud)[idx]);
                }
                cluster->width    = cluster->size();
                cluster->height   = 1;
                cluster->is_dense = true;
                result.push_back(cluster);
            }
        }
        clusters_[index] = result;
    }

    void PCLClustering::get_clusters(pcl::PCLPointCloud2& pointcloud2, size_t index)
    {
        if (index >= clusters_.size() || clusters_[index].empty()) return;

        pcl::PointCloud<pcl::PointXYZ> merged;
        for (const auto& cloud : clusters_[index])
        {
            merged += *cloud;
        }
        pcl::toPCLPointCloud2(merged, pointcloud2);
    }

    void PCLClustering::get_clusters(sensor_msgs::msg::PointCloud2& cloud_ros, size_t index)
    {
        pcl::PCLPointCloud2 pcl2;
        get_clusters(pcl2, index);
        pcl_conversions::fromPCL(pcl2, cloud_ros);
    }

    void PCLClustering::get_clusters(visualization_msgs::msg::MarkerArray& cloud_markers)
    {
        cloud_markers.markers.clear();

        for (size_t i = 0; i < clusters_.size(); ++i)
        {
            for (size_t j = 0; j < clusters_[i].size(); ++j)
            {
                const auto& cloud = clusters_[i][j];
                if (!cloud || cloud->empty()) continue;

                double cx = 0.0, cy = 0.0, cz = 0.0;
                for (const auto& pt : cloud->points)
                {
                    cx += pt.x;
                    cy += pt.y;
                    cz += pt.z;
                }
                const double n = static_cast<double>(cloud->size());
                cx /= n; cy /= n; cz /= n;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.ns              = "pcl_clusters";
                marker.id              = static_cast<int>(j + i * 10000);
                marker.type            = visualization_msgs::msg::Marker::SPHERE;
                marker.action          = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = cx;
                marker.pose.position.y = cy;
                marker.pose.position.z = cz;
                marker.pose.orientation.w = 1.0;
                marker.scale.x         = 0.2;
                marker.scale.y         = 0.2;
                marker.scale.z         = 0.2;
                marker.color.r         = static_cast<float>((j * 50) % 256) / 255.0f;
                marker.color.g         = static_cast<float>((j * 100) % 256) / 255.0f;
                marker.color.b         = static_cast<float>((j * 150) % 256) / 255.0f;
                marker.color.a         = 1.0f;

                cloud_markers.markers.push_back(marker);
            }
        }
    }

    void SupervoxelClustering::supervoxel_clustering(size_t index)
    {
        if (index >= clusters_.size() || clusters_[index].empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
        for (auto& cloud : clusters_[index])
        {
            if (cloud->empty()) continue;

            pcl::SupervoxelClustering<pcl::PointXYZ> super(
                static_cast<float>(supervoxel_voxel_resolution_),
                static_cast<float>(supervoxel_seed_resolution_));
            super.setColorImportance(static_cast<float>(supervoxel_color_importance_));
            super.setSpatialImportance(static_cast<float>(supervoxel_spatial_importance_));
            super.setNormalImportance(static_cast<float>(supervoxel_normal_importance_));
            super.setInputCloud(cloud);

            std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr> supervoxel_clusters;
            super.extract(supervoxel_clusters);

            for (auto& sv_pair : supervoxel_clusters)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                *cluster = *sv_pair.second->voxels_;
                if (!cluster->empty())
                {
                    result.push_back(cluster);
                }
            }
        }
        clusters_[index] = result;
    }

} // namespace potbot_lib
