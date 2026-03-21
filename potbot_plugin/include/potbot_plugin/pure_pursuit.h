// Copyright 2024 potbot
#ifndef POTBOT_PLUGIN__PURE_PURSUIT_H_
#define POTBOT_PLUGIN__PURE_PURSUIT_H_

#include <potbot_lib/pure_pursuit.h>
#include <potbot_base/base_controller.h>
#include <potbot_lib/utility_ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_plugin/PurePursuitConfig.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>

namespace potbot_nav
{
  namespace controller
  {
    class PurePursuit: public potbot_base::Controller
    {
private:
      potbot_lib::controller::PurePursuit pure_pursuit_;
      ros::Publisher pub_lookahead_;
      std::string frame_id_global_ = "map";
      dynamic_reconfigure::Server < potbot_plugin::PurePursuitConfig > *dsrv_;

      void reconfigureCB(const potbot_plugin::PurePursuitConfig & param, uint32_t level);

      void getLookahead(visualization_msgs::Marker & marker_msg);
      void publishLookahead();

public:
      PurePursuit() {
      }
      ~PurePursuit() {
      }

      void initialize(std::string name, tf2_ros::Buffer * tf);

      void calculateCommand(geometry_msgs::Twist & cmd_vel);
      void setTargetPath(const std::vector < geometry_msgs::PoseStamped > & path_msg);

      bool reachedTarget();
    };
  } // namespace controller
}  // namespace potbot_nav

#endif  // POTBOT_PLUGIN__PURE_PURSUIT_H_
