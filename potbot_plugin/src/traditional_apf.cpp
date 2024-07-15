#include <potbot_plugin/traditional_apf.h>

PLUGINLIB_EXPORT_CLASS(potbot_nav::controller::TraditionalAPF, potbot_base::Controller)
namespace potbot_nav
{
    namespace controller
    {
        using namespace potbot_lib::utility;
        void TraditionalAPF::initialize(std::string name, tf2_ros::Buffer* tf)
        {
            tf_ = tf;
            ros::NodeHandle private_nh("~/" + name);

            pub_potential_vector_ = private_nh.advertise<visualization_msgs::MarkerArray>("potential_vector", 1);
            sub_scan_ = private_nh.subscribe("/robot_0/scan",1, &TraditionalAPF::laserScanCallback, this);

            dsrv_ = new dynamic_reconfigure::Server<potbot_plugin::TraditionalAPFConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_plugin::TraditionalAPFConfig>::CallbackType cb = boost::bind(&TraditionalAPF::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void TraditionalAPF::reconfigureCB(const potbot_plugin::TraditionalAPFConfig& param, uint32_t level)
        {
            weight_attraction_field_ = param.weight_attraction_field;
            weight_repulsion_field_ = param.weight_repulsion_field;
            distance_threshold_repulsion_field_ = param.distance_threshold_repulsion_field;
            visualization_potential_scale_ = param.visualization_potential_scale;
            max_linear_velocity_ = param.max_linear_velocity;
            max_angular_velocity_ = param.max_angular_velocity;
        }

        void TraditionalAPF::setTargetPose(const geometry_msgs::PoseStamped& pose_msg)
        {
            target_pose_ = pose_msg;
            potbot_lib::Pose p;
            p.position.x = pose_msg.pose.position.x;
            p.position.y = pose_msg.pose.position.y;
            p.rotation.z = tf2::getYaw(pose_msg.pose.orientation);
            // pid_.setTargetPoint(p);
        }

        void TraditionalAPF::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
        {
            if (path_msg.empty()) return;
            target_path_ = path_msg;
            setTargetPose(path_msg.back());
        }

        void TraditionalAPF::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            // potbot_lib::utility::to_agent(robot_pose_, pid_);
            if (reachedTarget()) return;
            // pid_.calculateCommand();
            // potbot_lib::utility::to_msg(pid_, robot_pose_);
            // cmd_vel = robot_pose_.twist.twist;

            calculatePotential();
            publishPotential();

            double v = potential_.potential.norm();
            double omega = atan2(potential_.potential[1],potential_.potential[0]) - tf2::getYaw(robot_pose_.pose.pose.orientation);
            v = std::min(v, max_linear_velocity_);
            v = std::max(v, -max_linear_velocity_);
            omega = std::min(omega, max_angular_velocity_);
            omega = std::max(omega, -max_angular_velocity_);

            cmd_vel.linear.x = v;
            cmd_vel.angular.z = omega;

        }

        bool TraditionalAPF::reachedTarget()
        {
            return get_distance(robot_pose_.pose.pose, target_pose_.pose) < 0.3;
        }

        void TraditionalAPF::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
        {
            scan_ = *msg;
        }

        void TraditionalAPF::calculatePotential()
        {
            double eta_g = weight_attraction_field_;
            double eta_o = weight_repulsion_field_;
            double rho_o = distance_threshold_repulsion_field_;

            Eigen::Vector2d robot = get_vector(robot_pose_.pose.pose.position);
            Eigen::Vector2d goal = get_vector(target_pose_.pose.position);

            potential_.repulsion = Eigen::Vector2d{0,0};
            for (size_t i = 0; i < scan_.ranges.size(); i++)
            {
                double r = scan_.ranges[i];
                if (!finite(r)) continue;
                
                double th = double(i)*scan_.angle_increment + scan_.angle_min;
                
                geometry_msgs::PoseStamped scan_obstacle;
                scan_obstacle.header = scan_.header;
                scan_obstacle.pose = get_pose(r*cos(th), r*sin(th));

                geometry_msgs::PoseStamped global_obstacle_pose = get_tf(*tf_, scan_obstacle, frame_id_global_);

                Eigen::Vector2d obstacle = get_vector(global_obstacle_pose.pose.position);

                double rho = (obstacle - robot).norm();
                if (rho > rho_o)
                {
                    potential_.repulsion += Eigen::Vector2d{0,0};
                }
                else
                {
                    ROS_INFO_STREAM(obstacle.transpose());
                    potential_.repulsion += eta_o*((robot - obstacle)/pow(rho,3))*(1/rho-1/rho_o);
                }
            }
            
            potential_.attraction = -eta_g*(robot - goal);

            potential_.potential = potential_.attraction + potential_.repulsion;
        }

        Eigen::Vector2d TraditionalAPF::get_visualize_vector(Eigen::Vector2d vec)
        {
            double scaling = visualization_potential_scale_;

            double d = vec.norm();
            double th = atan2(vec[1],vec[0]) - tf2::getYaw(robot_pose_.pose.pose.orientation);
            double x = d*cos(th);
            double y = d*sin(th);

            return scaling*Eigen::Vector2d{x,y};
        }

        void TraditionalAPF::publishPotential()
        {

            visualization_msgs::MarkerArray potentail_arrows;
            visualization_msgs::Marker attraction_arrow, repulsion_arrow, potential_arrow;

            attraction_arrow.header.frame_id    = frame_id_global_;
            attraction_arrow.header.stamp       = ros::Time::now();

            attraction_arrow.ns                 = "attraction";
            attraction_arrow.id                 = 0;

            attraction_arrow.type               = visualization_msgs::Marker::ARROW;
            attraction_arrow.action             = visualization_msgs::Marker::MODIFY;

            attraction_arrow.pose               = robot_pose_.pose.pose;
            // attraction_arrow.pose               = potbot_lib::utility::get_pose();

            attraction_arrow.scale.x            = 0.05;
            attraction_arrow.scale.y            = 0.1;
            attraction_arrow.scale.z            = 0.1;
            
            attraction_arrow.color              = potbot_lib::color::get_msg("red");
            attraction_arrow.color.a            = 1;

            attraction_arrow.points.push_back(get_point());
            attraction_arrow.points.push_back(get_point(get_visualize_vector(potential_.attraction)));
            
            potentail_arrows.markers.push_back(attraction_arrow);

            repulsion_arrow = attraction_arrow;
            repulsion_arrow.ns                 = "repulsion";
            repulsion_arrow.id                 = 1;
            repulsion_arrow.color              = potbot_lib::color::get_msg("blue");
            repulsion_arrow.color.a            = 1;
            repulsion_arrow.points[1]          = get_point(get_visualize_vector(potential_.repulsion));
            potentail_arrows.markers.push_back(repulsion_arrow);

            potential_arrow = attraction_arrow;
            potential_arrow.ns                 = "potential";
            potential_arrow.id                 = 2;
            potential_arrow.color              = potbot_lib::color::get_msg("purple");
            potential_arrow.color.a            = 1;
            potential_arrow.points[1]          = get_point(get_visualize_vector(potential_.potential));
            potentail_arrows.markers.push_back(potential_arrow);

            pub_potential_vector_.publish(potentail_arrows);
        }
    }
}