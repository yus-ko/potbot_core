#ifndef H_SCANCLUSTERING_
#define H_SCANCLUSTERING_

#include <potbot_lib/utility_ros.h>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

namespace potbot_lib{

    typedef struct {
        std::vector<ScanPoint> points;
        int id=0;
        int type=0;
        double x=0;
        double y=0;
        double radius=0;
        double width=0;
        double height=0;
        bool is_moving=false;
    } Segment;

    class ScanClustering{
        private:
            std::vector<Segment> clusters_;

        public:
            ScanClustering();
            ~ScanClustering(){};
            
            void setClusters(const sensor_msgs::LaserScan& scan);
            void setClusters(const potbot_msgs::ObstacleArray& obstaclearray);
            void getClusters(std::vector<Segment>& clusters_arg);
            void euclideanClustering();
            void segmentation();
            void toMarkerarray(visualization_msgs::MarkerArray& ma);
            void toObstaclearray(potbot_msgs::ObstacleArray& oa);

    };
}

#endif	// H_SCANCLUSTERING_