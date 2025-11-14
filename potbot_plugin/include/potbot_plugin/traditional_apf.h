#ifndef H_POTBOT_NAV_TRADITIONAL_APF_
#define H_POTBOT_NAV_TRADITIONAL_APF_

#include <potbot_lib/pid.h>
#include <potbot_base/base_controller.h>
#include <potbot_lib/utility_ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_plugin/TraditionalAPFConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>

namespace potbot_nav
{
    namespace controller
    {
        typedef struct
        {
            Eigen::Vector2d attraction, repulsion, potential;
        } PotentialVector;

        class TraditionalAPF : public potbot_base::Controller
        {
            private:
                std::string frame_id_global_ = "map";
                double weight_attraction_field_ = 0.1;
                double weight_repulsion_field_ = 0.1;
                double distance_threshold_repulsion_field_ = 0.5;

                double visualization_potential_scale_ = 5;
                double max_linear_velocity_ = 0.5;
                double max_angular_velocity_ = 1.0;

                ros::Publisher pub_potential_vector_;
                ros::Subscriber sub_scan_;

                PotentialVector potential_;
                sensor_msgs::LaserScan scan_;

                dynamic_reconfigure::Server<potbot_plugin::TraditionalAPFConfig> *dsrv_;

                void reconfigureCB(const potbot_plugin::TraditionalAPFConfig& param, uint32_t level); 
                void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

                Eigen::Vector2d get_visualize_vector(Eigen::Vector2d vec);

                void calculatePotential();
                void publishPotential();

            public:
                TraditionalAPF(){};
                ~TraditionalAPF(){};

                void initialize(std::string name, tf2_ros::Buffer* tf);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPose(const geometry_msgs::PoseStamped& pose_msg);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);

                bool reachedTarget();

        };
    }
}

#endif	// H_POTBOT_NAV_TRADITIONAL_APF_