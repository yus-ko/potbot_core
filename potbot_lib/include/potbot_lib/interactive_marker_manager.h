#ifndef H_INTERACTIVE_MARKER_MANAGER_
#define H_INTERACTIVE_MARKER_MANAGER_

#include <potbot_lib/utility_ros.h>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace potbot_lib{

    class InteractiveMarkerManager
    {
        private:
            std::string name_space_ = "", frame_id_global_ = "map";
            size_t interactive_marker_num_ = 1;
            std::vector<visualization_msgs::Marker> interactive_markers_;

            interactive_markers::InteractiveMarkerServer *imsrv_;
            interactive_markers::MenuHandler *menu_handler_;

            void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

        public:
            InteractiveMarkerManager(std::string name = "");
            ~InteractiveMarkerManager();

            void initInteractiveMarkers();
            void initInteractiveMarkerServer();

            std::vector<visualization_msgs::Marker>* getMarker();
    };
}

#endif	// H_INTERACTIVE_MARKER_MANAGER_