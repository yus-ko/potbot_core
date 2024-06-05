#ifndef _H_UTILITY_
#define _H_UTILITY_

#include <random>
#include <eigen3/Eigen/Dense>
#include <potbot_msgs/ObstacleArray.h>
#include <ros/ros.h>
// #include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <potbot/beego_encoder.h>

namespace potbot_lib{

    const int SUCCESS = 1;
    const int FAIL = 0;

    // const int MEGAROVER = 0;
    // const int TURTLEBOT3 = 1;
    // const int BEEGO = 2;

    const int DEAD_RECKONING = 0;
    const int PARTICLE_FILTER = 1;

    const int CSV_PATH = 0;
    const int POTENTIAL_METHOD = 1;

    namespace color{
        const int RED           = 0;
        const int GREEN         = 1;
        const int BLUE          = 2;
        const int YELLOW        = 3;
        const int LIGHT_BLUE    = 4;
        const int PURPLE        = 5;
        const int WHITE         = 6;
        const int BLACK         = 7;

        std_msgs::ColorRGBA get_msg(const int color_id = potbot_lib::color::RED);
        std_msgs::ColorRGBA get_msg(const std::string color_name);
    }

    typedef struct {
        int index=0;
        double x=0;
        double y=0;
        double r=0;
        double theta=0;
    } Point;

    namespace utility{
        void get_RPY(const geometry_msgs::Quaternion& orientation, double &roll, double &pitch, double &yaw);
        geometry_msgs::Quaternion get_Quat(const double roll = 0, const double pitch = 0, const double yaw = 0);
        geometry_msgs::Point get_Point(const double x = 0, const double y = 0, const double z = 0);
        geometry_msgs::Pose get_Pose(const double x = 0, const double y = 0, const double z = 0, const double roll = 0, const double pitch = 0, const double yaw = 0);
        geometry_msgs::Pose get_Pose(const geometry_msgs::Point& p, const double roll = 0, const double pitch = 0, const double yaw = 0);
        double get_Yaw(const geometry_msgs::Quaternion& orientation);

        double get_Distance(const geometry_msgs::Point& position1, const geometry_msgs::Point& position2);
        double get_Distance(const geometry_msgs::Pose& position1, const geometry_msgs::Pose& position2);
        double get_Distance(const geometry_msgs::PoseStamped& position1, const geometry_msgs::PoseStamped& position2);
        double get_Distance(const nav_msgs::Odometry& position1, const nav_msgs::Odometry& position2);

        void print_Pose(const geometry_msgs::Pose& pose);
        void print_Pose(const geometry_msgs::PoseStamped& pose);
        void print_Pose(const nav_msgs::Odometry& pose);

        geometry_msgs::PoseStamped get_tf(const tf2_ros::Buffer &buffer, const geometry_msgs::PoseStamped& pose_in, const std::string target_frame_id);
        geometry_msgs::PointStamped get_tf(const tf2_ros::Buffer &buffer, const geometry_msgs::PointStamped& point_in, const std::string target_frame_id);
        geometry_msgs::PoseStamped get_tf(const tf2_ros::Buffer &buffer, const nav_msgs::Odometry& pose_in, const std::string target_frame_id);
        void get_tf(const tf2_ros::Buffer &buffer, const potbot_msgs::Obstacle& obstacle_in, const std::string target_frame_id, potbot_msgs::Obstacle& obstacle_out);
        void get_tf(const tf2_ros::Buffer &buffer, const potbot_msgs::ObstacleArray& obscales_in, const std::string target_frame_id, potbot_msgs::ObstacleArray& obscales_out);
        void get_tf(const tf2_ros::Buffer &buffer, const nav_msgs::Path& path_in, const std::string target_frame_id, nav_msgs::Path& path_out);

        geometry_msgs::PoseStamped get_Pose_from_tf(const tf2_ros::Buffer &buffer, const std::string source_frame_id, const std::string target_frame_id);
        int get_WorldCoordinate(std::string target_frame, ros::Time time, geometry_msgs::PoseStamped &Wcood, tf2_ros::Buffer &buffer);

        geometry_msgs::Point get_MapCoordinate(int index, nav_msgs::MapMetaData info);
        int get_MapIndex(double x, double y, const nav_msgs::MapMetaData& info);

        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::Point& position);
        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::Pose& position);
        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& position);
        int get_PathIndex(const nav_msgs::Path& path, const nav_msgs::Odometry& position);

        void associate_obstacle(potbot_msgs::ObstacleArray& obstacle_input, const potbot_msgs::ObstacleArray& obstacle_compare, const tf2_ros::Buffer &buffer);

        void find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest);

        int get_index(const std::vector<Eigen::Vector2d>& vec, const Eigen::Vector2d& value);

        void to_msg(const std::vector<Eigen::Vector2d>& vectors, nav_msgs::Path& msg);

        typedef struct {
            bool running                = false;
            double begin_time           = 0.0;
            double end_time             = 0.0;
            double duration             = 0.0;
        } TimerInfo;

        class Timer{
            protected:
                std::map<std::string, TimerInfo> times_;
            public:
                void start(const std::string timer_name, const double time = -1);
                void start(const std::vector<std::string> timer_names);
                void stop(const std::string timer_name, const double time = -1);
                void stop(const std::vector<std::string> timer_names = {});
                void print_time(const std::string timer_names);
                void print_time(const std::vector<std::string> timer_names = {});
        };
    }
}

#endif	// _H_UTILITY_