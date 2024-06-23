#include <potbot_lib/scan_clustering.h>

namespace potbot_lib{

    ScanClustering::ScanClustering()
    {

    }

    void ScanClustering::setClusters(const sensor_msgs::LaserScan& scan)
    {
        clusters_.clear();
        Segment clus;
        size_t p_idx = 0, range_idx = 0;
        for (const auto& range : scan.ranges)
        {
            if (scan.range_min <= range && range <= scan.range_max)
            {
                ScanPoint p;
                p.index = p_idx++;
                p.theta = (double)range_idx * scan.angle_increment + scan.angle_min;
                p.r = range;   //minは足さない
                p.x = p.r * cos(p.theta);
                p.y = p.r * sin(p.theta);
                clus.points.push_back(p);
            }
            range_idx++;
        }
        clusters_.push_back(clus);
        
    }

    void ScanClustering::setClusters(const potbot_msgs::ObstacleArray& obstaclearray)
    {
        clusters_.resize(obstaclearray.data.size());
        size_t p_idx = 0;
        for (size_t i = 0; i < clusters_.size(); i++)
        {
            clusters_[i].x                      = obstaclearray.data[i].pose.position.x;
            clusters_[i].y                      = obstaclearray.data[i].pose.position.y;
            clusters_[i].width                  = obstaclearray.data[i].scale.x;
            clusters_[i].height                 = obstaclearray.data[i].scale.y;
            clusters_[i].id                     = obstaclearray.data[i].id;
            clusters_[i].is_moving              = obstaclearray.data[i].is_moving;

            clusters_[i].points.resize(obstaclearray.data[i].points.size());
            for (size_t j = 0; j < clusters_[i].points.size(); j++)
            {
                clusters_[i].points[j].index    = p_idx++;
                clusters_[i].points[j].x        = obstaclearray.data[i].points[j].x;
                clusters_[i].points[j].y        = obstaclearray.data[i].points[j].y;
                clusters_[i].points[j].r        = sqrt(pow(obstaclearray.data[i].points[j].x,2) + pow(obstaclearray.data[i].points[j].y,2));
                clusters_[i].points[j].theta    = atan2(obstaclearray.data[i].points[j].y, obstaclearray.data[i].points[j].x);
            }
        }
    }
    
    void ScanClustering::getClusters(std::vector<Segment>& clusters_arg)
    {
        clusters_arg = clusters_;
    }

    void ScanClustering::segmentation()
    {

    }

    // void ScanClustering::euclidean_clustering()
    // {
    //     size_t size = clusters_[0].points.size();
    //     std::vector<bool> checked(size);
    //     std::fill(checked.begin(), checked.end(), false);

    //     for(size_t i = 0; i < clusters_[0].points.size(); i++)
    //     {
    //         const Point& pi = clusters_[0].points[i];
    //         for(size_t j = 0; j < clusters_[0].points.size(); j++)
    //         {
    //             if (checked[j]) continue;
    //             const Point& pj = clusters_[0].points[j];
    //             double distance = sqrt(pow(pi.x - pj.x,2) + pow(pi.y - pj.y,2));
    //             if (distance <= 0.3)
    //             {

    //             }
    //         }

    //         checked[i] = true;
    //     }

    // }

    void ScanClustering::euclideanClustering()
    {
        int size = clusters_[0].points.size();
        bool start = false;
        Segment seg;
        for (int i = 0; i < size; i++)
        {
            
            if(!start)
            {
                start = true;
                seg.points.clear();
            }

            ScanPoint &p = clusters_[0].points[i];

            if (!seg.points.empty())
            {    
                ScanPoint &p_next = seg.points.back();

                double distance = sqrt(pow(p.x - p_next.x,2) + pow(p.y - p_next.y,2));
                if (distance <= 0.3)
                {
                    seg.points.push_back(p);
                }
                else
                {
                    start = false;
                    clusters_.push_back(seg);
                    continue;
                }
            }
            else
            {
                seg.points.push_back(p);
            }

            if (i == size - 1 && start)
            {
                clusters_.push_back(seg);
            }
        }

        clusters_.erase(clusters_.begin());

        int id = 0;
        for (auto& clus : clusters_)
        {

            std::vector<double> vec_x, vec_y;
            //x のみを抽出
            std::transform(clus.points.begin(), clus.points.end(), std::back_inserter(vec_x), [](const ScanPoint& p) { return p.x; });
            //y のみを抽出
            std::transform(clus.points.begin(), clus.points.end(), std::back_inserter(vec_y), [](const ScanPoint& p) { return p.y; });

            // xの最小値を取得
            auto min_x_itr = std::min_element(vec_x.begin(), vec_x.end());
            double min_x = (min_x_itr != vec_x.end()) ? *min_x_itr : 0.0;
            // xの最大値を取得
            auto max_x_itr = std::max_element(vec_x.begin(), vec_x.end());
            double max_x = (max_x_itr != vec_x.end()) ? *max_x_itr : 0.0;

            // yの最小値を取得
            auto min_y_itr = std::min_element(vec_y.begin(), vec_y.end());
            double min_y = (min_y_itr != vec_y.end()) ? *min_y_itr : 0.0;
            // yの最大値を取得
            auto max_y_itr = std::max_element(vec_y.begin(), vec_y.end());
            double max_y = (max_y_itr != vec_y.end()) ? *max_y_itr : 0.0;

            clus.id = id++;
            clus.type = visualization_msgs::Marker::SPHERE;
            clus.width = abs(max_x - min_x);
            clus.height = abs(max_y - min_y);
            clus.x = min_x + clus.width/2.0;
            clus.y = min_y + clus.height/2.0;
        }

    }

    void ScanClustering::toMarkerarray(visualization_msgs::MarkerArray& ma)
    {
        ma.markers.clear();
        int points_id = clusters_.back().id + 1;
        for (const auto& clus : clusters_)
        {
            visualization_msgs::Marker marker;

            marker.ns = "scan2d/centor";
            marker.id = clus.id;
            marker.lifetime = ros::Duration(1);

            marker.type = clus.type;
            marker.action = visualization_msgs::Marker::MODIFY;

            marker.pose = potbot_lib::utility::get_Pose(clus.x, clus.y, 0,0,0,0);

            marker.scale.x = clus.width;
            marker.scale.y = clus.height;
            marker.scale.z = 0.001;

            marker.color = potbot_lib::color::get_msg(clus.id);
            marker.color.a = 0.3;
            
            ma.markers.push_back(marker);

            visualization_msgs::Marker points;
            points.ns = "scan2d/points";
            points.id = points_id++;
            points.lifetime = ros::Duration(1);
            points.type = visualization_msgs::Marker::SPHERE_LIST;
            points.action = visualization_msgs::Marker::ADD;
            points.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);
            points.scale.x = 0.01;
            points.scale.y = 0.01;
            points.scale.z = 0.01;
            points.color = marker.color;

            for (const auto& point : clus.points)
            {
                points.points.push_back(potbot_lib::utility::get_Point(point.x, point.y, 0));
                points.colors.push_back(points.color);
            }
            ma.markers.push_back(points);
        }
    }

    void ScanClustering::toObstaclearray(potbot_msgs::ObstacleArray& oa)
    {
        oa.data.clear();
        for (const auto& clus : clusters_)
        {
            potbot_msgs::Obstacle obstacle;
            obstacle.header = oa.header;
            obstacle.id = clus.id;
            obstacle.is_moving = clus.is_moving;
            obstacle.pose = potbot_lib::utility::get_Pose(clus.x, clus.y, 0,0,0,0);

            obstacle.scale.x = clus.width;
            obstacle.scale.y = clus.height;
            obstacle.scale.z = 0.001;

            for (const auto& p : clus.points)
            {
                geometry_msgs::Point p_msg;
                p_msg.x = p.x;
                p_msg.y = p.y;
                p_msg.z = 0.0;
                obstacle.points.push_back(p_msg);
            }
            
            oa.data.push_back(obstacle);
        }
    }

}
