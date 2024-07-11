#include <potbot_plugin/pure_pursuit.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::PurePursuit, potbot_base::Controller)
namespace potbot_nav
{
    namespace controller
    {
        void PurePursuit::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
            pub_lookahead_ = private_nh.advertise<visualization_msgs::Marker>("lookahead", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::PurePursuitConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::PurePursuitConfig>::CallbackType cb = boost::bind(&PurePursuit::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void PurePursuit::reconfigureCB(const potbot_plugin::PurePursuitConfig& param, uint32_t level)
        {
            pure_pursuit_.setMargin(param.stop_margin_angle, param.stop_margin_distance);
            pure_pursuit_.setLimit(param.max_linear_velocity, param.max_angular_velocity);
            pure_pursuit_.setDistanceToLookaheadPoint(param.distance_to_lookahead_point);
            pure_pursuit_.setNormalize(param.normalize);
        }

        void PurePursuit::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            potbot_lib::utility::to_agent(robot_pose_, pure_pursuit_);
            if (reachedTarget()) return;
            pure_pursuit_.calculateCommand();
            publishLookahead();
            potbot_lib::utility::to_msg(pure_pursuit_, robot_pose_);
            cmd_vel = robot_pose_.twist.twist;
        }

        void PurePursuit::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            if (path_msg.empty()) return;
            target_path_ = path_msg;
            target_pose_ = path_msg.back();
            std::vector<potbot_lib::Pose> path;
            for (const auto pose : path_msg)
            {
                potbot_lib::Pose p;
                p.position.x = pose.pose.position.x;
                p.position.y = pose.pose.position.y;
                p.rotation.z = potbot_lib::utility::get_Yaw(pose.pose.orientation);
                path.push_back(p);
            }
            pure_pursuit_.setTargetPath(path);
        }

        void PurePursuit::getLookahead(visualization_msgs::Marker& marker_msg)
        {
            auto lookahead = pure_pursuit_.getLookahead();

            marker_msg.ns                    = "LookAhead";
            marker_msg.id                    = 0;
            marker_msg.lifetime              = ros::Duration(0);

            marker_msg.type                  = visualization_msgs::Marker::SPHERE;
            marker_msg.action                = visualization_msgs::Marker::MODIFY;
            
            marker_msg.pose                  = potbot_lib::utility::get_Pose(lookahead.position.x, lookahead.position.y, 0, 0, 0, lookahead.rotation.z);

            marker_msg.scale.x               = 0.08;
            marker_msg.scale.y               = 0.08;
            marker_msg.scale.z               = 0.08;

            marker_msg.color                 = potbot_lib::color::get_msg(potbot_lib::color::RED);
            marker_msg.color.a               = 0.5;
            
        }

        void PurePursuit::publishLookahead()
        {
            visualization_msgs::Marker marker_msg;
            getLookahead(marker_msg);
            marker_msg.header.frame_id = frame_id_global_;
            marker_msg.header.stamp = ros::Time::now();
            pub_lookahead_.publish(marker_msg);
        }

        bool PurePursuit::reachedTarget()
        {
            return pure_pursuit_.reachedTarget();
        }
    }
}