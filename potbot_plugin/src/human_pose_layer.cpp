#include "potbot_plugin/human_pose_layer.hpp"

#include <algorithm>
#include <limits>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/time.h"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace potbot_plugin
{

    HumanPoseLayer::HumanPoseLayer() : need_recalculation_(false)
    {
    }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void HumanPoseLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        detections_sub_ = node->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections_3d", qos,
            [this](const yolo_msgs::msg::DetectionArray::SharedPtr msg)
            {
                detectionsCallback(msg);
            });

        need_recalculation_ = false;
        current_ = true;
    }

    void HumanPoseLayer::detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
    {
        std::scoped_lock<std::mutex> lock(detections_mutex_);
        last_detections_ = msg;
        need_recalculation_ = true;

        RCLCPP_DEBUG(logger_,
                    "HumanPoseLayer received %zu detections on /yolo/detections_3d",
                    msg->detections.size());

        auto target_frame = layered_costmap_->getGlobalFrameID();
        last_keypoints_.clear();

        yolo_msgs::msg::DetectionArray::SharedPtr detections;
        {
            detections = last_detections_;
        }

        if (!detections)
        {
            return;
        }

        for (const auto &info_detected : detections->detections)
        {
            for (const auto &keypoint : info_detected.keypoints3d.data)
            {
                geometry_msgs::msg::Point map_point;
                if (!transformToMapFrame(keypoint.point, detections->header.frame_id,
                                         detections->header.stamp, target_frame, map_point))
                {
                    continue;
                }
                last_keypoints_.push_back(map_point);
            }
        }
    }

    bool HumanPoseLayer::transformToMapFrame(
        const geometry_msgs::msg::Point &src_point,
        const std::string &source_frame,
        const rclcpp::Time &stamp,
        const std::string &target_frame,
        geometry_msgs::msg::Point &out_point)
    {
        auto node = node_.lock();
        if (!node)
        {
            return false;
        }

        geometry_msgs::msg::PointStamped src;
        src.header.frame_id = source_frame;
        src.header.stamp = stamp;
        src.point = src_point;

        geometry_msgs::msg::PointStamped dst;
        try
        {
            dst = tf_->transform(src, target_frame, tf2::durationFromSec(0.1));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,
                                    "TF transform failed for keypoint to %s: %s",
                                    target_frame.c_str(), ex.what());
            return false;
        }

        out_point = dst.point;
        return true;
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void HumanPoseLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            // For some reason when I make these -<double>::max() it does not
            // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
            // -<float>::max() instead.
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        }
    }

    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void HumanPoseLayer::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                            "nav2_costmap_2d"),
                        "HumanPoseLayer::onFootprintChanged(): num footprint points: %lu",
                        layered_costmap_->getFootprint().size());
    }

    // The method is called when costmap recalculation is required.
    // It updates the costmap within its window bounds.
    // Inside this method the costmap gradient is generated and is writing directly
    // to the resulting costmap master_grid without any merging with previous layers.
    void HumanPoseLayer::updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid, 
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        unsigned char *master_array = master_grid.getCharMap();

        for (const auto &keypoint : last_keypoints_)
        {
            unsigned int mx, my;
            if (!master_grid.worldToMap(keypoint.x, keypoint.y, mx, my))
            {
                continue;
            }

            auto index = master_grid.getIndex(mx, my);
            master_array[index] = LETHAL_OBSTACLE;
        }

        return;
    }

} // namespace potbot_plugin

// This is the macro allowing a potbot_plugin::HumanPoseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potbot_plugin::HumanPoseLayer, nav2_costmap_2d::Layer)