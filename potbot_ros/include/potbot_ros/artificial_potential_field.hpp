#ifndef HPP_POTBOT_ROS_ARTIFICIAL_POTENTIAL_FIELD_
#define HPP_POTBOT_ROS_ARTIFICIAL_POTENTIAL_FIELD_

#include <potbot_ros/utility.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <potbot_lib/field.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace potbot_lib{

    class ArtificialPotentialFieldROS{
        private:
            rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_potential_field_;
            
            std::string frame_id_global_ = "map";

            // void reconfigureCB(const potbot_lib::PotentialFieldConfig& param, uint32_t level); 
        protected:
            ArtificialPotentialField* apf_;
        public:
            
            ArtificialPotentialFieldROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr _node);
            ~ArtificialPotentialFieldROS(){};

            void initNode(const std::string &name);
            void initPotentialField(const nav2_costmap_2d::Costmap2D* costmap);
            void initPotentialField(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
            void initPotentialField();

            ArtificialPotentialField* getApf();

            void setFrameIdGlobal(std::string frame_id);
            std::string getFrameIdGlobal();

            void setGoal(const geometry_msgs::msg::PoseStamped& goal);

            void setRobot(const geometry_msgs::msg::Pose &robot);
            void setRobot(const geometry_msgs::msg::PoseStamped& robot);
            void setRobot(const nav_msgs::msg::Odometry &robot);

            void setObstacle(const visualization_msgs::msg::Marker &obs);
            void setObstacle(const std::vector<visualization_msgs::msg::Marker> &obs);
            void setObstacle(const geometry_msgs::msg::Point &obs);
            void setObstacle(const std::vector<geometry_msgs::msg::Point> &obs);
            void setObstacle(const geometry_msgs::msg::PointStamped &obs);
            void setObstacle(const std::vector<geometry_msgs::msg::PointStamped> &obs);
            void setObstacle(const geometry_msgs::msg::Pose &obs);
            void setObstacle(const std::vector<geometry_msgs::msg::Pose> &obs);
            void setObstacle(const geometry_msgs::msg::PoseStamped& obs);
            void setObstacle(const std::vector<geometry_msgs::msg::PoseStamped> &obs);

            void setObstacle(const nav2_costmap_2d::Costmap2D *costmap);

            void clearObstacles();
            void createPotentialField();

            void publishPotentialField();
    };
}

#endif // HPP_POTBOT_ROS_ARTIFICIAL_POTENTIAL_FIELD_