#include <potbot_lib/diff_drive_controller_ros.h>

namespace potbot_lib{

    namespace controller{

        DiffDriveControllerROS::DiffDriveControllerROS(std::string name)
        {
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
            pub_lookahead_ = private_nh.advertise<visualization_msgs::Marker>("lookahead", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_lib::ControllerConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_lib::ControllerConfig>::CallbackType cb = boost::bind(&DiffDriveControllerROS::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void DiffDriveControllerROS::reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level)
        {
            setGain( param.gain_p, param.gain_i, param.gain_d);
            setMargin(	param.stop_margin_angle, param.stop_margin_distance);
            setLimit( param.max_linear_velocity, param.max_linear_velocity);
            setDistanceToLookaheadPoint(param.distance_to_lookahead_point);
        }

        void DiffDriveControllerROS::setTarget(const geometry_msgs::Pose& pose_msg)
        {
            DiffDriveController::setTarget(	pose_msg.position.x,
                                                pose_msg.position.y,
                                                tf2::getYaw(pose_msg.orientation));
        }

        void DiffDriveControllerROS::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            done_init_pose_alignment_ = false;
            set_init_pose_ = false;
            target_path_.clear();
            size_t idx = 0;
            target_path_index_ = 0;
            for (const auto pose : path_msg)
            {
                Pose p;
                p.position.x = pose.pose.position.x;
                p.position.y = pose.pose.position.y;
                p.rotation.z = potbot_lib::utility::get_Yaw(pose.pose.orientation);
                target_path_.push_back(p);
            }
            lookahead_ = &target_path_.front();
        }

        void DiffDriveControllerROS::setTargetPath(const nav_msgs::Path& path_msg)
        {
            setTargetPath(path_msg.poses);
        }

        void DiffDriveControllerROS::getLookahead(visualization_msgs::Marker& marker_msg)
        {
            
            if (lookahead_ == nullptr) return;

            marker_msg.ns                    = "LookAhead";
            marker_msg.id                    = 0;
            marker_msg.lifetime              = ros::Duration(0);

            marker_msg.type                  = visualization_msgs::Marker::SPHERE;
            marker_msg.action                = visualization_msgs::Marker::MODIFY;
            
            marker_msg.pose                  = potbot_lib::utility::get_Pose(lookahead_->position.x, lookahead_->position.y, 0, 0, 0, lookahead_->rotation.y);

            marker_msg.scale.x               = 0.08;
            marker_msg.scale.y               = 0.08;
            marker_msg.scale.z               = 0.08;

            marker_msg.color                 = potbot_lib::color::get_msg(potbot_lib::color::RED);
            marker_msg.color.a               = 0.5;
            
        }

        void DiffDriveControllerROS::publishLookahead()
        {
            visualization_msgs::Marker marker_msg;
            getLookahead(marker_msg);
            marker_msg.header.frame_id = frame_id_global_;
            marker_msg.header.stamp = ros::Time::now();
            pub_lookahead_.publish(marker_msg);
        }

    }
}