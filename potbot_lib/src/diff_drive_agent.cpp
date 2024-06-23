#include <potbot_lib/diff_drive_agent.h>

namespace potbot_lib{

    void DiffDriveAgent::toMsg(nav_msgs::Odometry& odom_msg)
    {
        odom_msg.header.stamp           = ros::Time::now();
        odom_msg.pose.pose.position.x   = x;
        odom_msg.pose.pose.position.y   = y;
        odom_msg.pose.pose.position.z   = 0.0;
        odom_msg.pose.pose.orientation  = utility::get_Quat(0,0,yaw);
        odom_msg.twist.twist.linear.x   = v;
        odom_msg.twist.twist.linear.y   = 0.0;
        odom_msg.twist.twist.linear.z   = 0.0;
        odom_msg.twist.twist.angular.x  = 0.0;
        odom_msg.twist.twist.angular.y  = 0.0;
        odom_msg.twist.twist.angular.z  = omega;
    }

    void DiffDriveAgent::setMsg(const geometry_msgs::Pose& pose_msg)
    {
        x                               = pose_msg.position.x;
        y                               = pose_msg.position.y;
        yaw                             = tf2::getYaw(pose_msg.orientation);
    }

    void DiffDriveAgent::setMsg(const geometry_msgs::PoseStamped& pose_msg)
    {
        setMsg(pose_msg.pose);
    }

    void DiffDriveAgent::setMsg(const nav_msgs::Odometry& odom_msg)
    {
        setMsg(odom_msg.pose.pose);
        v                               = odom_msg.twist.twist.linear.x;
        omega                           = odom_msg.twist.twist.angular.z;
    }

    void DiffDriveAgent::update()
    {
        yaw                             += omega*deltatime;
        x                               += v*deltatime*cos(yaw);
        y                               += v*deltatime*sin(yaw);
    }

    double DiffDriveAgent::getDistance(const Point& p)
    {
        return sqrt(pow(x-p.x,2) + pow(y-p.y,2));
    }

    double DiffDriveAgent::getDistance(const Pose& p)
    {
        return getDistance(p.position);
    }

    double DiffDriveAgent::getAngle(const Point& p)
    {
        return atan2(p.y - y, p.x - x);
    }

    double DiffDriveAgent::getAngle(const Pose& p)
    {
        return getAngle(p.position);
    }
}