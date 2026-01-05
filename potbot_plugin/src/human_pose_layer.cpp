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

    HumanPoseLayer::HumanPoseLayer()
        : last_min_x_(-std::numeric_limits<float>::max()),
          last_min_y_(-std::numeric_limits<float>::max()),
          last_max_x_(std::numeric_limits<float>::max()),
                    last_max_y_(std::numeric_limits<float>::max()),
                    need_recalculation_(false)
    {
    }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration and initialization
    // of need_recalculation_ variable.
    void
    HumanPoseLayer::onInitialize()
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

    void
    HumanPoseLayer::detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
    {
        std::scoped_lock<std::mutex> lock(detections_mutex_);
        last_detections_ = msg;
        need_recalculation_ = true;

        RCLCPP_DEBUG(logger_,
                    "HumanPoseLayer received %zu detections on /yolo/detections_3d",
                    msg->detections.size());

        // for (const auto &info_detected : last_detections_->detections)
        // {
        //     RCLCPP_INFO_STREAM(logger_, info_detected.class_name);
        //     // for (const auto &keypoint : info_detected.keypoints3d.data)
        //     // {
        //     //     RCLCPP_INFO(logger_, "%f, %f, %f", keypoint.point.x, keypoint.point.y, keypoint.point.z);
        //     // }
        //     RCLCPP_INFO(logger_, "%f, %f, %f",
        //                 info_detected.bbox3d.center.position.x,
        //                 info_detected.bbox3d.center.position.y,
        //                 info_detected.bbox3d.center.position.z);
        // }
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    HumanPoseLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            // For some reason when I make these -<double>::max() it does not
            // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
            // -<float>::max() instead.
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_recalculation_ = false;
        }
        else
        {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }

    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void
    HumanPoseLayer::onFootprintChanged()
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
    void
    HumanPoseLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
        int max_i,
        int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        // master_array - is a direct pointer to the resulting master_grid.
        // master_grid - is a resulting costmap combined from all layers.
        // By using this pointer all layers will be overwritten!
        // To work with costmap layer and merge it with other costmap layers,
        // please use costmap_ pointer instead (this is pointer to current
        // costmap layer grid) and then call one of updates methods:
        // - updateWithAddition()
        // - updateWithMax()
        // - updateWithOverwrite()
        // - updateWithTrueOverwrite()
        // In this case using master_array pointer is equal to modifying local costmap_
        // pointer and then calling updateWithTrueOverwrite():
        unsigned char *master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        auto target_frame = layered_costmap_->getGlobalFrameID();

        yolo_msgs::msg::DetectionArray::SharedPtr detections;
        {
            std::scoped_lock<std::mutex> lock(detections_mutex_);
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
                geometry_msgs::msg::PointStamped src_point;
                src_point.header.frame_id = detections->header.frame_id;
                src_point.header.stamp = detections->header.stamp;
                src_point.point = keypoint.point;

                geometry_msgs::msg::PointStamped map_point;
                try
                {
                    map_point = tf_->transform(src_point, target_frame, tf2::durationFromSec(0.1));
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                         "TF transform failed for keypoint to %s: %s",
                                         target_frame.c_str(), ex.what());
                    continue;
                }

                unsigned int mx, my;
                if (!master_grid.worldToMap(map_point.point.x, map_point.point.y, mx, my))
                {
                    continue;
                }

                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
            }

            // unsigned int mx, my;
            // master_grid.worldToMap(
            //     info_detected.bbox3d.center.position.x,
            //     info_detected.bbox3d.center.position.y, 
            //     mx, my);
            // auto index = master_grid.getIndex(mx, my);
            // master_array[index] = LETHAL_OBSTACLE;
        }

        return;

        // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
        // These variables are used to update the costmap only within this window
        // avoiding the updates of whole area.
        //
        // Fixing window coordinates with map size if necessary.
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        // Simply computing one-by-one cost per each cell
        int gradient_index;
        for (int j = min_j; j < max_j; j++)
        {
            // Reset gradient_index each time when reaching the end of re-calculated window
            // by OY axis.
            gradient_index = 0;
            for (int i = min_i; i < max_i; i++)
            {
                int index = master_grid.getIndex(i, j);
                // setting the gradient cost
                unsigned char cost = (LETHAL_OBSTACLE - gradient_index * GRADIENT_FACTOR) % 255;
                if (gradient_index <= GRADIENT_SIZE)
                {
                    gradient_index++;
                }
                else
                {
                    gradient_index = 0;
                }
                master_array[index] = cost;
            }
        }
    }

} // namespace potbot_plugin

// This is the macro allowing a potbot_plugin::HumanPoseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(potbot_plugin::HumanPoseLayer, nav2_costmap_2d::Layer)