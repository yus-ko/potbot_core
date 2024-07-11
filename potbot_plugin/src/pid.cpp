#include <potbot_plugin/pid.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::PID, potbot_base::Controller)
namespace potbot_nav
{
    namespace controller
    {
        void PID::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            ros::NodeHandle private_nh("~/" + name);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::PIDConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::PIDConfig>::CallbackType cb = boost::bind(&PID::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void PID::reconfigureCB(const potbot_plugin::PIDConfig& param, uint32_t level)
        {
            pid_.setMargin(param.stop_margin_angle, param.stop_margin_distance);
            pid_.setLimit(param.max_linear_velocity, param.max_angular_velocity);
            pid_.setGain(param.gain_p, param.gain_i, param.gain_d);
        }

        void PID::setTargetPose(const geometry_msgs::PoseStamped& pose_msg)
        {
            target_pose_ = pose_msg;
            potbot_lib::Pose p;
            p.position.x = pose_msg.pose.position.x;
            p.position.y = pose_msg.pose.position.y;
            p.rotation.z = tf2::getYaw(pose_msg.pose.orientation);
            pid_.setTargetPoint(p);
        }

        void PID::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            if (path_msg.empty()) return;
            target_path_ = path_msg;
            setTargetPose(path_msg.back());
        }

        void PID::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            potbot_lib::utility::to_agent(robot_pose_, pid_);
            if (reachedTarget()) return;
            pid_.calculateCommand();
            potbot_lib::utility::to_msg(pid_, robot_pose_);
            cmd_vel = robot_pose_.twist.twist;
        }

        bool PID::reachedTarget()
        {
            return pid_.reachedTarget();
        }
    }
}