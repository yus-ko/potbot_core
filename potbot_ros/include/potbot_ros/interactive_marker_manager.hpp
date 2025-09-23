#ifndef H_INTERACTIVE_MARKER_MANAGER_
#define H_INTERACTIVE_MARKER_MANAGER_

#include <memory>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <potbot_lib/py_string.hpp>
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
            bool is_initialized_ = false;
            std::string marker_file_ = "";
        
            rclcpp::TimerBase::SharedPtr timer_;

            std::string name_space_ = "", frame_id_global_ = "map";
            std::vector<std::string> mesh_resource_files_;
            std::map<std::string, VisualMarker> controllable_markers_;

            std::shared_ptr<interactive_markers::InteractiveMarkerServer> imsrv_;
            std::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

            visualization_msgs::msg::Marker default_visual_marker_;

            visualization_msgs::msg::InteractiveMarkerControl 
                movement_controller_, 
                movement_controller_axis_z_, 
                rotation_controller_, 
                rotation_controller_axis_x_, 
                rotation_controller_axis_y_,
                rotation_controller_axis_z_,
                scale_controller_, 
                scale_controller_axis_x_,
                scale_controller_axis_y_,
                scale_controller_axis_z_,
                scale_controller_axis_xyz_;

            std::map<std::string, interactive_markers::MenuHandler::EntryHandle> entry_handles_;

            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
            virtual rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

            virtual void initializeController();
            virtual void initializeParameter();
            virtual void initializeMenu();
            virtual void initializeMarker(std::string yaml_path = "", bool set_default = true);

            virtual void initializeMarkerServer(const std::map<std::string, VisualMarker> &markers, const std::vector<std::string> &marker_with_controller = {});

            void editorChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string mode);
            virtual void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void resetPositionZ(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            virtual void changeRotation(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void resetRotation(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void changeScale(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void resetScale(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            void typeChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type, std::string mesh_resource = "");
            void colorChangeTo(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std_msgs::msg::ColorRGBA color);
            virtual YAML::Node saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            YAML::Node getYamlNode(const VisualMarker &visual_marker);
            VisualMarker getVisualMarker(const YAML::Node &yaml_node);
            VisualMarker getVisualMarker(std::string name, const Pose &init_pose=Pose());
            VisualMarker getVisualMarker(std::string name, const visualization_msgs::msg::Marker &vis_marker, const Pose &init_pose=Pose());
            virtual std::string duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
            virtual void deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

            void setMenuVisibles(bool visible);

            std::string getCopyName(std::string original_name);
            std::string getMarkerName(std::string controller_name);

            virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        public:
            InteractiveMarkerManager(std::string name="marker", std::string node_namespace="");
            ~InteractiveMarkerManager(){};

            void registerFeedback(std::string marker_name,
                const interactive_markers::InteractiveMarkerServer::FeedbackCallback &feedbck_func);

            void addMarker(std::string name, const Pose &init_pose=Pose());
            void addMarker(std::string name, const visualization_msgs::msg::Marker &vis_marker, 
                            const Pose &init_pose=Pose());
            void addMarker(std::string name, const VisualMarker &visual_marker);
            void addMarker(VisualMarker &visual_marker);

            geometry_msgs::msg::Pose getMarkerPose(std::string name);
    };
}

#endif	// H_INTERACTIVE_MARKER_MANAGER_