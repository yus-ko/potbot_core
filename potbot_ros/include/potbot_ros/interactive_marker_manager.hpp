#ifndef H_INTERACTIVE_MARKER_MANAGER_
#define H_INTERACTIVE_MARKER_MANAGER_

#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <potbot_lib/interpolate.hpp>
#include <potbot_ros/utility.hpp>

namespace potbot_lib{

    typedef struct{
        visualization_msgs::msg::InteractiveMarker marker;
        visualization_msgs::msg::InteractiveMarker controller;
    } VisualMarker;

    class InteractiveMarkerManager : public rclcpp_lifecycle::LifecycleNode
    {
        protected:
        
            rclcpp::TimerBase::SharedPtr timer_;

            std::string name_space_ = "", frame_id_global_ = "map";
            std::map<std::string, VisualMarker> controllable_markers_;

            std::shared_ptr<interactive_markers::InteractiveMarkerServer> imsrv_;
            std::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

            visualization_msgs::msg::InteractiveMarkerControl 
                movement_controller_, 
                rotation_controller_, 
                rotation_controller_axis_x_, 
                rotation_controller_axis_y_,
                rotation_controller_axis_z_,
                scale_controller_, 
                scale_controller_axis_x_,
                scale_controller_axis_y_,
                scale_controller_axis_z_;
            
            interactive_markers::InteractiveMarkerServer::FeedbackCallback 
                function_change_position_,
                function_change_rotation_;

            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
            rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

            void initializeController();
            void initializeParameter();
            void initializeMenu();
            void initializeMarker(std::string yaml_path = "", bool set_default = true);

            void editorChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string mode);
            void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void changeRotation(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void changeScale(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void typeChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type);
            void saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

            int getMarkerId(std::string marker_name);

        public:
            InteractiveMarkerManager(std::string name="marker", std::string node_namespace="");
            ~InteractiveMarkerManager(){};

            void registerFeedback(std::string marker_name,
                const interactive_markers::InteractiveMarkerServer::FeedbackCallback &feedbck_func);

            geometry_msgs::msg::Pose getMarkerPose(std::string name);
    };
}

#endif	// H_INTERACTIVE_MARKER_MANAGER_