#include <potbot_lib/base_controller.h>

namespace potbot_lib
{
    namespace controller
    {

        void BaseController::setRobot(const nav_msgs::Odometry& pose_msg)
        {
            setMsg(pose_msg);
        }

        void BaseController::setTargetPose(const geometry_msgs::Pose& pose_msg)
        {
            DiffDriveController::setTarget(	pose_msg.position.x,
                                            pose_msg.position.y,
                                            tf2::getYaw(pose_msg.orientation));
        }

        void BaseController::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            target_path_.clear();
            for (const auto pose : path_msg)
            {
                Pose p;
                p.position.x = pose.pose.position.x;
                p.position.y = pose.pose.position.y;
                p.rotation.z = potbot_lib::utility::get_Yaw(pose.pose.orientation);
                target_path_.push_back(p);
            }
        }

        void BaseController::getRobot(nav_msgs::Odometry& pose_msg)
        {
            toMsg(pose_msg);
        }
    }
}