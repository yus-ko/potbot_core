#ifndef HPP_POTBOT_ROS_APF_PATH_PLANNER_
#define HPP_POTBOT_ROS_APF_PATH_PLANNER_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.hpp>

#include <potbot_ros/utility.hpp>
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_ros/artificial_potential_field.hpp>

namespace potbot_lib{

    namespace path_planner{

        void getPathMsgFromCsv(nav_msgs::msg::Path& path_msg,const std::string& csv_fullpath);

        class APFPathPlannerROS : public APFPathPlanner{
            private:
                rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
                rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_, pub_raw_path_;
                std::string frame_id_global_ = "map";

            public:
                // APFPathPlannerROS(ArtificialPotentialField* apf);
                APFPathPlannerROS(std::shared_ptr<ArtificialPotentialFieldROS> apf_ros);
                ~APFPathPlannerROS(){};
                
                void getLoopEdges(visualization_msgs::msg::MarkerArray& msg);

                void getPath(std::vector<geometry_msgs::msg::PoseStamped> &msg);
                void getPath(nav_msgs::msg::Path &msg);

                void publishPath();
                void publishRawPath();
        };

    }
}

#endif // HPP_POTBOT_ROS_APF_PATH_PLANNER_