#ifndef H_POTENTIALFIELD_ROS_
#define H_POTENTIALFIELD_ROS_

#include <potbot_lib/utility_ros.h>
#include <potbot_lib/artificial_potential_field.h>
#include <potbot_lib/field.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/PotentialFieldConfig.h>

namespace potbot_lib{

    class ArtificialPotentialFieldROS{
        private:
            ros::Publisher pub_potential_field_;
            std::string frame_id_global_ = "map";
            dynamic_reconfigure::Server<potbot_lib::PotentialFieldConfig> *dsrv_;

            void reconfigureCB(const potbot_lib::PotentialFieldConfig& param, uint32_t level); 
        protected:
            ArtificialPotentialField* apf_;
        public:
            
            ArtificialPotentialFieldROS(std::string name = "potential_field");
            ~ArtificialPotentialFieldROS(){};

            void initNode(std::string name);
            void initPotentialField(costmap_2d::Costmap2D* costmap);
            void initPotentialField(costmap_2d::Costmap2DROS* costmap_ros);
            void initPotentialField();

            ArtificialPotentialField* getApf();

            void setFrameIdGlobal(std::string frame_id);
            std::string getFrameIdGlobal();

            void setGoal(const geometry_msgs::PoseStamped& goal);

            void setRobot(const geometry_msgs::Pose& robot);
            void setRobot(const geometry_msgs::PoseStamped& robot);
            void setRobot(const nav_msgs::Odometry& robot);
            
            void setObstacle(const visualization_msgs::Marker& obs);
            void setObstacle(const std::vector<visualization_msgs::Marker>& obs);
            void setObstacle(const geometry_msgs::Point& obs);
            void setObstacle(const std::vector<geometry_msgs::Point>& obs);
            void setObstacle(const geometry_msgs::PointStamped& obs);
            void setObstacle(const std::vector<geometry_msgs::PointStamped>& obs);
            void setObstacle(const geometry_msgs::Pose& obs);
            void setObstacle(const std::vector<geometry_msgs::Pose>& obs);
            void setObstacle(const geometry_msgs::PoseStamped& obs);
            void setObstacle(const std::vector<geometry_msgs::PoseStamped>& obs);

            void setObstacle(costmap_2d::Costmap2D* costmap);

            void clearObstacles();
            void createPotentialField();

            void publishPotentialField();
    };
}

#endif	// H_POTENTIALFIELD_ROS_