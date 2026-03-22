#ifndef HPP_POTBOT_ROS_SCAN_CLUSTERING_
#define HPP_POTBOT_ROS_SCAN_CLUSTERING_

#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <potbot_lib/utility.hpp>

namespace potbot_lib
{

    struct Segment
    {
        std::vector<ScanPoint> points;
        int id = 0;
        int type = 0;
        double x = 0;       // 重心x
        double y = 0;       // 重心y
        double radius = 0;  // バウンディング半径
        double width = 0;
        double height = 0;
        bool is_moving = false;
    };

    class ScanClustering
    {
    protected:
        std::vector<Segment> clusters_;
        double cluster_threshold_ = 0.2;
        double min_scan_range_ = 0.1;
        double max_scan_range_ = 10.0;

        void euclideanClustering(const std::vector<ScanPoint> & points);

    public:
        ScanClustering();
        ~ScanClustering() {};

        void setClusters(const sensor_msgs::msg::LaserScan & scan);

        void getClusters(std::vector<Segment> & clusters_arg);

        void euclideanClustering();
        void segmentation();

        void toMarkerarray(visualization_msgs::msg::MarkerArray & ma);
    };

} // namespace potbot_lib

#endif // HPP_POTBOT_ROS_SCAN_CLUSTERING_
