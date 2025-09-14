#ifndef H_INTERACTIVE_MARKER_MANAGER_
#define H_INTERACTIVE_MARKER_MANAGER_

#include <boost/bind/bind.hpp>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

// #include <potbot_lib/Save.h>
// #include <std_srvs/Empty.h>

#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <potbot_lib/interpolate.hpp>
#include <potbot_ros/utility.hpp>

namespace potbot_lib{

    // typedef struct{
    //     visualization_msgs::msg::Marker marker;
    //     std::vector<geometry_msgs::msg::PoseStamped> trajectory;
    //     bool trajectory_recording = false;
    //     u_int8_t trajectory_marker_type = visualization_msgs::msg::Marker::LINE_STRIP;
    //     std::string trajectory_interpolation_method = "none";
    // } VisualMarker;

    typedef struct{
        visualization_msgs::msg::InteractiveMarker marker;
        visualization_msgs::msg::InteractiveMarker controller;
    } VisualMarker;

    class InteractiveMarkerManager
    {
        private:
            rclcpp::Node* parent_node_;

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_trajectory_;

            // ros::ServiceServer srv_save_marker_trajectory_, srv_clear_marker_trajectory_;

            std::string name_space_ = "", frame_id_global_ = "map";
            size_t interactive_marker_num_ = 1;
            std::vector<VisualMarker> visual_markers_;
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

            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
            rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

            void initializeController();
            void initializeParameter();
            void initializeMenu();
            void initializeMarker(std::string yaml_path = "", bool set_default = true);

            void markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void editorChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string mode);
            void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void changeRotation(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void changeScale(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void typeChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type);
            void saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

            void interpolateTrajectory(size_t id);

            void publishMarkerTrajectory();
            
            // bool serviceSaveMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp);
            // bool serviceClearMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp);

            int getMarkerId(std::string marker_name);

        public:
            InteractiveMarkerManager(std::string name, rclcpp::Node* node);
            ~InteractiveMarkerManager(){};

            std::vector<VisualMarker>* getVisualMarker();
            visualization_msgs::msg::InteractiveMarker getMarker(std::string name);
            std::vector<visualization_msgs::msg::InteractiveMarker> getAllMarkers();
    };
}

#endif	// H_INTERACTIVE_MARKER_MANAGER_