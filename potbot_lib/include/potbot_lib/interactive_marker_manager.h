#ifndef H_INTERACTIVE_MARKER_MANAGER_
#define H_INTERACTIVE_MARKER_MANAGER_

#include <potbot_lib/interpolate.h>
#include <potbot_lib/utility_ros.h>

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <dynamic_reconfigure/server.h>
#include <potbot_lib/MarkerManagerConfig.h>

namespace potbot_lib{

    typedef struct{
        visualization_msgs::Marker marker;
        std::vector<geometry_msgs::PoseStamped> trajectory;
        bool trajectory_recording = false;
        u_int8_t trajectory_marker_type = visualization_msgs::Marker::LINE_STRIP;
        std::string trajectory_interpolation_method = "none";
    } VisualMarker;

    class InteractiveMarkerManager
    {
        private:
            ros::Publisher pub_marker_trajectory_;

            std::string name_space_ = "", frame_id_global_ = "map";
            size_t interactive_marker_num_ = 1;
            // std::vector<visualization_msgs::InteractiveMarker> interactive_markers_;
            std::vector<VisualMarker> visual_markers_;

            interactive_markers::InteractiveMarkerServer *imsrv_;
            interactive_markers::MenuHandler *menu_handler_;

            dynamic_reconfigure::Server<potbot_lib::MarkerManagerConfig> *dsrv_;

            void reconfigureCB(const potbot_lib::MarkerManagerConfig& param, uint32_t level); 
            void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

            void interpolateTrajectory(size_t id);

            void publishMarkerTrajectory();

        public:
            InteractiveMarkerManager(std::string name = "");
            ~InteractiveMarkerManager(){};

            std::vector<VisualMarker>* getVisualMarker();
    };
}

#endif	// H_INTERACTIVE_MARKER_MANAGER_