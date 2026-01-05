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
#include "yolo_msgs/msg/detection_array.hpp"

namespace potbot_plugin
{

    class HumanPoseLayer : public nav2_costmap_2d::Layer
    {
    public:
        HumanPoseLayer();

        virtual void onInitialize();
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, double *min_x,
            double *min_y,
            double *max_x,
            double *max_y);
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

        rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
        yolo_msgs::msg::DetectionArray::SharedPtr last_detections_;
        std::mutex detections_mutex_;

        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

        // Indicates that the entire gradient should be recalculated next time.
        std::atomic_bool need_recalculation_;

        // Size of gradient in cells
        int GRADIENT_SIZE = 20;
        // Step of increasing cost per one cell in gradient
        int GRADIENT_FACTOR = 10;
    };

} // namespace potbot_plugin

#endif // POTBOT_NAV_HUMAN_POSE_LAYER_HPP_