#ifndef POTBOT_NAV_HUMAN_POSE_LAYER_HPP_
#define POTBOT_NAV_HUMAN_POSE_LAYER_HPP_

#include <atomic>
#include <mutex>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace potbot_plugin
{

    class HumanPoseLayer : public nav2_costmap_2d::Layer
    {
    public:
        HumanPoseLayer();

        virtual void onInitialize();
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, 
            double *min_x, double *min_y, double *max_x, double *max_y);
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j);

        virtual void reset()
        {
            return;
        }

        virtual void onFootprintChanged();

        virtual bool isClearable() { return false; }

    private:
        void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
        bool transformToMapFrame(
            const geometry_msgs::msg::Point &src_point,
            const std::string &source_frame,
            const rclcpp::Time &stamp,
            const std::string &target_frame,
            geometry_msgs::msg::Point &out_point);

        rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
        yolo_msgs::msg::DetectionArray::SharedPtr last_detections_;
        std::vector<geometry_msgs::msg::Point> last_keypoints_;
        std::mutex detections_mutex_;

        // Indicates that the entire gradient should be recalculated next time.
        std::atomic_bool need_recalculation_;
    };

} // namespace potbot_plugin

#endif // POTBOT_NAV_HUMAN_POSE_LAYER_HPP_