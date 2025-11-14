#include <potbot_plugin/time_state.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::TimeState, potbot_base::Controller)
namespace potbot_nav
{
    namespace controller
    {
        using namespace potbot_lib::utility;

        void TimeState::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            tf_ = tf;

            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::TimeStateConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::TimeStateConfig>::CallbackType cb = boost::bind(&TimeState::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void TimeState::reconfigureCB(const potbot_plugin::TimeStateConfig& param, uint32_t level)
        {
            controller_.setMargin(param.stop_margin_angle, param.stop_margin_distance);
            controller_.setLimit(param.max_linear_velocity, param.max_angular_velocity);
            controller_.setWeight(param.weight_y, param.weight_yaw);
        }

        void TimeState::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            int idx = get_path_index(target_path_, robot_pose_);
            
            broadcast_frame(tf_broadcaster_, frame_id_global_, "target_path", target_path_[idx].pose);
            geometry_msgs::PoseStamped p = get_tf(*tf_, robot_pose_, "target_path");
            nav_msgs::Odometry robot_pose_from_path_frame;
            robot_pose_from_path_frame.pose.pose = p.pose;

            to_agent(robot_pose_, controller_);
            if (reachedTarget()) return;

            to_agent(robot_pose_from_path_frame, controller_);
            controller_.calculateCommand();
            to_msg(controller_, robot_pose_from_path_frame);
            cmd_vel = robot_pose_from_path_frame.twist.twist;
        }

        void TimeState::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            if (path_msg.empty()) return;
            target_path_ = path_msg;
            set_path_orientation(target_path_);
            target_pose_ = path_msg.back();
            std::vector<potbot_lib::Pose> path;
            for (const auto pose : path_msg)
            {
                potbot_lib::Pose p;
                p.position.x = pose.pose.position.x;
                p.position.y = pose.pose.position.y;
                p.rotation.z = tf2::getYaw(pose.pose.orientation);
                path.push_back(p);
            }
            controller_.setTargetPath(path);
        }

        bool TimeState::reachedTarget()
        {
            return controller_.reachedTarget();
        }
    }
}