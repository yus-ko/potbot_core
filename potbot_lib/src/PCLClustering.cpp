#include <potbot_lib/PCLClustering.h>

namespace potbot_lib
{

    PCLClustering::PCLClustering()
    {
        point_cloud_.resize(1);
    }

    void PCLClustering::set_clusters(const potbot_msgs::ObstacleArray& obstaclearray)
    {
        // for (const auto& obses: obstaclearray.data)
        // {
        //     pcl::PointXYZ pcl_point;
        //     pcl_point.x = obses.pose.position.x;
        //     pcl_point.y = obses.pose.position.y;
        //     pcl_point.z = obses.pose.position.z;
        //     point_cloud_->push_back(pcl_point);
        // }
    }

    void PCLClustering::set_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, size_t index)
    {
        point_cloud_[index] = point_cloud;
    }

    void PCLClustering::get_clusters(pcl::PCLPointCloud2& pointcloud2, size_t index)
    {
        pcl::toPCLPointCloud2(*(point_cloud_[index]), pointcloud2);
    }

    void PCLClustering::get_clusters(sensor_msgs::PointCloud2& cloud_ros, size_t index)
    {
        pcl::PCLPointCloud2 pointcloud2;
        get_clusters(pointcloud2, index);
        pcl_conversions::fromPCL(pointcloud2, cloud_ros);
    }

    void PCLClustering::get_clusters(visualization_msgs::MarkerArray& cloud_markers)
    {
        cloud_markers.markers.clear();
        for(const auto& cluster : point_cloud_)
        {
            visualization_msgs::Marker marker_centor;
            pcl_conversions::fromPCL(cluster->header, marker_centor.header);
            marker_centor.ns = "euclidean/centor";
            marker_centor.id = cloud_markers.markers.size();
            marker_centor.lifetime = ros::Duration(0);

            marker_centor.type = visualization_msgs::Marker::SPHERE;
            marker_centor.action = visualization_msgs::Marker::ADD;
            
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*(cluster), minPt, maxPt);
            marker_centor.pose.position.x = (maxPt.x - minPt.x)/2 + minPt.x;
            marker_centor.pose.position.y = (maxPt.y - minPt.y)/2 + minPt.y;
            marker_centor.pose.position.z = (maxPt.z - minPt.z)/2 + minPt.z;

            marker_centor.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);

            marker_centor.scale.x = abs(maxPt.x - minPt.x);
            marker_centor.scale.y = abs(maxPt.y - minPt.y);
            marker_centor.scale.z = abs(maxPt.z - minPt.z);

            marker_centor.color = potbot_lib::color::get_msg(marker_centor.id);
            cloud_markers.markers.push_back(marker_centor);

            visualization_msgs::Marker marker_points = marker_centor;
            marker_points.ns = "euclidean/points";
            marker_points.id = cloud_markers.markers.size();
            marker_points.type = visualization_msgs::Marker::SPHERE_LIST;
            marker_points.pose.position.x = 0;
            marker_points.pose.position.y = 0;
            marker_points.pose.position.z = 0;
            marker_points.scale.x = down_sampling_voxel_size_;
            marker_points.scale.y = down_sampling_voxel_size_;
            marker_points.scale.z = down_sampling_voxel_size_;
            
            for(const auto& cluster_point : cluster->points)
            {

                geometry_msgs::Point point;
                point.x = cluster_point.x;
                point.y = cluster_point.y;
                point.z = cluster_point.z;
                
                marker_points.points.push_back(point);
                marker_points.colors.push_back(marker_centor.color);
            
            }
            cloud_markers.markers.push_back(marker_points);

        }
    }

    void PCLClustering::get_clusters(potbot_msgs::ObstacleArray& obstaclearray)
    {
        obstaclearray.data.clear();
        pcl_conversions::fromPCL(point_cloud_.front()->header, obstaclearray.header);

        for(const auto& cluster : point_cloud_)
        {
            potbot_msgs::Obstacle cluster_msg;
            pcl_conversions::fromPCL(cluster->header, cluster_msg.header);
            // marker_centor.id = cloud_markers.markers.size();
            
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*(cluster), minPt, maxPt);
            cluster_msg.pose.position.x = (maxPt.x - minPt.x)/2 + minPt.x;
            cluster_msg.pose.position.y = (maxPt.y - minPt.y)/2 + minPt.y;
            cluster_msg.pose.position.z = (maxPt.z - minPt.z)/2 + minPt.z;

            cluster_msg.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);

            cluster_msg.scale.x = abs(maxPt.x - minPt.x);
            cluster_msg.scale.y = abs(maxPt.y - minPt.y);
            cluster_msg.scale.z = abs(maxPt.z - minPt.z);
            
            for(const auto& cluster_point : cluster->points)
            {
                geometry_msgs::Point point;
                point.x = cluster_point.x;
                point.y = cluster_point.y;
                point.z = cluster_point.z;
                
                cluster_msg.points.push_back(point);
            }
            obstaclearray.data.push_back(cluster_msg);
        }

    }
    

    void PCLClustering::down_sampling(size_t index)
    {
        //データセットのダウンサンプリング
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (point_cloud_[index]);
        vg.setLeafSize(down_sampling_voxel_size_, down_sampling_voxel_size_, down_sampling_voxel_size_);
        vg.filter (*(point_cloud_[index]));
    }

    void PCLClustering::plane_removal(size_t index)
    {
        // 平面モデル作成のためのパラメーター
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (plane_removal_distance_threshold_);

        int nr_points = (int) point_cloud_[index]->size ();
        while (point_cloud_[index]->size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (point_cloud_[index]);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                // ROS_ERROR("Could not estimate a planar model for the given dataset.");
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (point_cloud_[index]);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            // ROS_INFO("PointCloud representing the planar component: %d data points.", cloud_plane->size());

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            //cloud_fをメンバ変数からローカル変数に変更
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f {new pcl::PointCloud<pcl::PointXYZ>};
            extract.filter (*cloud_f);
            *(point_cloud_[index]) = *cloud_f;
        }
    }

    void PCLClustering::euclidean_clustering(size_t index)
    {
        /*kd-treeクラスを宣言*/
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        /*探索する点群をinput*/
        tree->setInputCloud(point_cloud_[index]);
        /*クラスタリング後のインデックスが格納されるベクトル*/
        std::vector<pcl::PointIndices> cluster_indices;
        /*クラスタリング手法*/
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        /*距離の閾値を設定*/
        ece.setClusterTolerance(euclidean_cluster_tolerance_);
        /*各クラスタのメンバの最小数を設定*/
        ece.setMinClusterSize(euclidean_min_cluster_size_);
        /*各クラスタのメンバの最大数を設定*/
        ece.setMaxClusterSize(point_cloud_[index]->points.size());
        /*探索方法を設定*/
        ece.setSearchMethod(tree);
        /*クラスリング対象の点群をinput*/
        ece.setInputCloud(point_cloud_[index]);
        /*クラスリング実行*/
        ece.extract(cluster_indices);

        // ROS_INFO("cluster_indices.size() = %d", cluster_indices.size());

        /*dividing（クラスタごとに点群を分割）*/
        pcl::ExtractIndices<pcl::PointXYZ> ei;
        ei.setInputCloud(point_cloud_[index]);
        ei.setNegative(false);
        point_cloud_.clear();
        for(size_t i=0;i<cluster_indices.size();i++){
            /*extract*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
            *tmp_clustered_indices = cluster_indices[i];
            ei.setIndices(tmp_clustered_indices);
            ei.filter(*tmp_clustered_points);
            /*input*/
            point_cloud_.push_back(tmp_clustered_points);
        }
    }

}

namespace potbot_lib
{

    void PCLSuperVoxel::get_clusters(visualization_msgs::MarkerArray& cloud_markers)
    {
        cloud_markers = marker_array_;
    }

    void PCLSuperVoxel::supervoxel_clustering(size_t index)
    {
        marker_array_.markers.clear();

        pcl::SupervoxelClustering<pcl::PointXYZ> super (supervoxel_voxel_resolution_, supervoxel_seed_resolution_);
        super.setInputCloud (point_cloud_[index]);
        super.setColorImportance (supervoxel_color_importance_);
        super.setSpatialImportance (supervoxel_spatial_importance_);
        super.setNormalImportance (supervoxel_normal_importance_);

        std::map <std::uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr > supervoxel_clusters;

        // ROS_INFO("Extracting supervoxels!");
        super.extract (supervoxel_clusters);
        // ROS_INFO("Found %d supervoxels", supervoxel_clusters.size());

        // ROS_INFO("Getting supervoxel adjacency");
        std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency (supervoxel_adjacency);

        //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
        for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
        {
            //First get the label
            std::uint32_t supervoxel_label = label_itr->first;
            //Now get the supervoxel corresponding to the label
            pcl::Supervoxel<pcl::PointXYZ>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

            //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
            pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
            pcl::PointCloud<pcl::PointXYZ> adjacent_supervoxel_voxels;
            for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
            {
                pcl::Supervoxel<pcl::PointXYZ>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
                adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
                adjacent_supervoxel_voxels = *(neighbor_supervoxel->voxels_);
            }
            
            visualization_msgs::Marker marker_centor;
            pcl_conversions::fromPCL(point_cloud_[index]->header, marker_centor.header);
            marker_centor.ns = "superVoxel/centor";
            marker_centor.lifetime = ros::Duration(0);
            marker_centor.type = visualization_msgs::Marker::CUBE;
            marker_centor.action = visualization_msgs::Marker::ADD;
            marker_centor.pose.orientation = potbot_lib::utility::get_Quat(0,0,0);
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (adjacent_supervoxel_voxels, minPt, maxPt);
            if(abs(maxPt.x - minPt.x) == 0) marker_centor.scale.x = supervoxel_voxel_resolution_;
            else                            marker_centor.scale.x = abs(maxPt.x - minPt.x);
            if(abs(maxPt.y - minPt.y) == 0) marker_centor.scale.y = supervoxel_voxel_resolution_;
            else                            marker_centor.scale.y = abs(maxPt.y - minPt.y);  
            if(abs(maxPt.z - minPt.z) == 0) marker_centor.scale.x = supervoxel_voxel_resolution_;
            else                            marker_centor.scale.z = abs(maxPt.z - minPt.z);
            
            for(int i = 0; i < adjacent_supervoxel_centers.size(); i++)
            {
                marker_centor.id = marker_array_.markers.size();
                marker_centor.pose.position.x = adjacent_supervoxel_centers[i].x;
                marker_centor.pose.position.y = adjacent_supervoxel_centers[i].y;
                marker_centor.pose.position.z = adjacent_supervoxel_centers[i].z;
                marker_centor.color = potbot_lib::color::get_msg(marker_centor.id);
                marker_array_.markers.push_back(marker_centor);

                if(i == 0)
                {
                    visualization_msgs::Marker marker_voxels = marker_centor;
                    marker_voxels.ns = "superVoxel/points";
                    marker_voxels.id = marker_array_.markers.size();

                    marker_voxels.type = visualization_msgs::Marker::CUBE_LIST;

                    marker_voxels.pose.position.x = 0;
                    marker_voxels.pose.position.y = 0;
                    marker_voxels.pose.position.z = 0;

                    marker_voxels.scale.x = supervoxel_voxel_resolution_;
                    marker_voxels.scale.y = supervoxel_voxel_resolution_;
                    marker_voxels.scale.z = supervoxel_voxel_resolution_;
                    
                    for(int j = 0; j < adjacent_supervoxel_voxels.size(); j++)
                    {
                        geometry_msgs::Point point;
                        point.x = adjacent_supervoxel_voxels[j].x;
                        point.y = adjacent_supervoxel_voxels[j].y;
                        point.z = adjacent_supervoxel_voxels[j].z;
                        
                        marker_voxels.points.push_back(point);
                        marker_voxels.colors.push_back(marker_voxels.color);
                    }
                    marker_array_.markers.push_back(marker_voxels);
                }
            }

            //Move iterator forward to next label
            label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
        }

        point_cloud_.clear();
    }
}