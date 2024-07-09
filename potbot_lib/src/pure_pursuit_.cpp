#include <potbot_lib/pure_pursuit.h>
#include <pluginlib/class_loader.h>
// #include <test_pkg/test_lib.h>

PLUGINLIB_EXPORT_CLASS(potbot_lib::controller::PurePursuit, potbot_lib::controller::BaseController)
namespace potbot_lib
{
    namespace controller
    {

        void PurePursuit::initialize(std::string name)
        {
            ros::NodeHandle private_nh("~/" + name);
            private_nh.getParam("frame_id_global",           frame_id_global_);
            pub_lookahead_ = private_nh.advertise<visualization_msgs::Marker>("lookahead", 1);

            dsrv_ = new dynamic_reconfigure::Server<potbot_lib::ControllerConfig>(private_nh);
            dynamic_reconfigure::Server<potbot_lib::ControllerConfig>::CallbackType cb = boost::bind(&PurePursuit::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            // pluginlib::ClassLoader<nav_core::BaseLocalPlanner>
            pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader("nav_core", "nav_core::BaseLocalPlanner");
            // std::string plugin_name = "potbot_nav/PotbotLocalPlanner";
            std::string plugin_name = "dwa_local_planner/DWAPlannerROS";
            // n.getParam("controller_name", plugin_name);
            try
            {
                ddr_ = loader.createInstance(plugin_name);
                // ddr_ = loader.createInstance(plugin_name);
                // ddr_->initialize("controller");
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("failed to load plugin. Error: %s", ex.what());
            }

            // pluginlib::ClassLoader<test_lib::TestLib> loader2("test_pkg", "test_lib::TestLib");
            // try
            // {
            //     loader2.createInstance("test_lib/plugin");
            // }
            // catch(pluginlib::PluginlibException& ex)
            // {
            //     ROS_ERROR("failed to load plugin. Error: %s", ex.what());
            // }
        }

        void PurePursuit::reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level)
        {
            setGain( param.gain_p, param.gain_i, param.gain_d);
            setMargin(	param.stop_margin_angle, param.stop_margin_distance);
            setLimit( param.max_linear_velocity, param.max_linear_velocity);
            setDistanceToLookaheadPoint(param.distance_to_lookahead_point);
            reset_path_index_ = param.reset_path_index;
        }

        void PurePursuit::calculateCommand(geometry_msgs::Twist& cmd_vel)
        {
            if (reachedTarget()) return;
            purePursuitController();
            // normalizedPurePursuit();
            publishLookahead();
            nav_msgs::Odometry robot;
            toMsg(robot);
            cmd_vel = robot.twist.twist;
        }

        void PurePursuit::setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg)
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

            if (reset_path_index_)
            {
                done_init_pose_alignment_ = false;
                set_init_pose_ = false;
                target_path_index_ = 0;
                lookahead_ = &target_path_.front();
            }
            
        }

        void PurePursuit::setTargetPath(const nav_msgs::Path& path_msg)
        {
            setTargetPath(path_msg.poses);
        }

        void PurePursuit::getLookahead(visualization_msgs::Marker& marker_msg)
        {
            
            if (lookahead_ == nullptr) return;

            marker_msg.ns                    = "LookAhead";
            marker_msg.id                    = 0;
            marker_msg.lifetime              = ros::Duration(0);

            marker_msg.type                  = visualization_msgs::Marker::SPHERE;
            marker_msg.action                = visualization_msgs::Marker::MODIFY;
            
            marker_msg.pose                  = potbot_lib::utility::get_Pose(lookahead_->position.x, lookahead_->position.y, 0, 0, 0, lookahead_->rotation.z);

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

        void PurePursuit::purePursuitController()
        {
            v=0;
            omega=0;

            if (target_path_.empty()) return;

            if (getDistance(target_path_.back().position) <= distance_to_lookahead_point_)
            {
                line_following_process_ = PROCESS_STOP;
                return;
            }

            size_t target_path_size = target_path_.size();

            Pose* sub_goal = &target_path_.front();
            double l_d;
            while(true)
            {
                sub_goal = &target_path_[target_path_index_];
                l_d = getDistance(*sub_goal);
                if (l_d <= distance_to_lookahead_point_)
                {
                    target_path_index_++;
                    if (target_path_index_ > target_path_size-1)
                    {
                        target_path_index_ = target_path_size-1;
                        return;
                        // break;
                    }
                }
                else
                {
                    lookahead_ = sub_goal;
                    break;
                }
            }

            ROS_DEBUG("process: %d, index: %d, size: %d", (int)line_following_process_, (int)target_path_index_, (int)target_path_size);

            if(initialize_pose_ && !done_init_pose_alignment_)
            {
                double init_angle = getTargetPathInitAngle();

                if (abs(init_angle - yaw) > stop_margin_angle_ || l_d > 2*distance_to_lookahead_point_)
                {
                    if (!set_init_pose_)
                    {
                        set_init_pose_ = true;
                        setTarget(lookahead_->position.x, lookahead_->position.y, init_angle);
                        line_following_process_ = RETURN_TO_TARGET_PATH;
                    }
                }
                else
                {
                    line_following_process_ = FOLLOWING_PATH;
                }

                if (line_following_process_ == RETURN_TO_TARGET_PATH)
                {
                    pidControl();
                    applyLimit();
                }
                
            }
            else
            {
                line_following_process_ = FOLLOWING_PATH;
            }

            if (line_following_process_ == FOLLOWING_PATH && target_path_index_ < target_path_size)
            {   
                done_init_pose_alignment_ = true;
                double alpha = getAngle(*lookahead_) - yaw;
                v = max_linear_velocity_;
                omega = 2.0*v*sin(alpha)/l_d;
                applyLimit();
            }
        }

        void PurePursuit::normalizedPurePursuit()
        {
            v=0;
            omega=0;

            if (target_path_.empty()) return;

            size_t target_path_size = target_path_.size();

            double d_min = 1e100;
            for (size_t i = 0; i < target_path_size; i++)
            {
                double d = getDistance(target_path_[i]);
                if (d < d_min)
                {
                    d_min = d;
                    target_path_index_ = i;
                }
            }

            double d_sum = 0;
            for (size_t i = target_path_index_; i < target_path_size; i++)
            {
                if (i > 0)
                {
                    d_sum += hypot(target_path_[i].position.x - target_path_[i-1].position.x,target_path_[i].position.y - target_path_[i-1].position.y);
                    if (d_sum > distance_to_lookahead_point_)
                    {
                        lookahead_ = &target_path_[i];
                        break;
                    }
                }
            }
            double l_d = getDistance(*lookahead_);

            // double l_d;
            // while(true)
            // {
            //     Pose* sub_goal = &target_path_[target_path_index_];
            //     l_d = getDistance(*sub_goal);
            //     if (l_d <= distance_to_lookahead_point_)
            //     {
            //         target_path_index_++;
            //         if (target_path_index_ > target_path_size-1)
            //         {
            //             target_path_index_ = target_path_size-1;
            //             return;
            //         }
            //     }
            //     else
            //     {
            //         lookahead_ = sub_goal;
            //         break;
            //     }
            // }

            ROS_DEBUG("index: %d, size: %d", (int)target_path_index_, (int)target_path_size);

            double alpha = getAngle(*lookahead_) - yaw;
            v = max_linear_velocity_/(abs(alpha)+1.0);
            double nv = v/max_linear_velocity_;
            omega = 2.0*nv*sin(alpha)/l_d;
            applyLimit();

            // double alpha = getAngle(*lookahead_) - yaw;
            // v = max_linear_velocity_;
            // omega = 2.0*v*sin(alpha)/l_d;
            // applyLimit();
        }

        bool PurePursuit::reachedTarget()
        {
            return abs(getDistance(target_point_)) <= distance_to_lookahead_point_ && target_path_index_ == target_path_.size()-1;
            // ROS_DEBUG("if reached index: %d, size: %d", target_path_index_, target_path_.size());
            // ROS_DEBUG_STREAM(bool(target_path_index_ == target_path_.size()-1));

            // if (target_path_index_ == target_path_.size()-1)
            // {
            //     ROS_INFO("true");
            //     return true;
            // }
            // else
            // {
            //     ROS_INFO("false");
            //     return false;
            // }
            

            // // return bool(target_path_index_ == target_path_.size()-1);
        }
    }
}