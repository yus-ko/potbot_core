/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <potbot_plugin/local_planner.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/Path.h>

#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(potbot_nav::PotbotLocalPlanner, nav_core::BaseLocalPlanner)

namespace potbot_nav
{

    PotbotLocalPlanner::PotbotLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    PotbotLocalPlanner::PotbotLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {

        // initialize the planner
        initialize(name, tf, costmap_ros);
    }

    void PotbotLocalPlanner::initialize(
        std::string name,
        tf2_ros::Buffer *tf,
        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized())
        {

            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            rot_stopped_velocity_ = 1e-2;
            trans_stopped_velocity_ = 1e-2;
            double sim_time, sim_granularity, angular_sim_granularity;
            int vx_samples, vtheta_samples;
            double path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
            bool holonomic_robot, dwa, simple_attractor, heading_scoring;
            double heading_scoring_timestep;
            double max_vel_x, min_vel_x;
            double backup_vel;
            double stop_time_buffer;
            std::string world_model_type;
            rotating_to_goal_ = false;

            // initialize the copy of the costmap the controller will use
            costmap_ = costmap_ros_->getCostmap();

            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();
            private_nh.param("prune_plan", prune_plan_, true);

            private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
            private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
            private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
            private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
            private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

            private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

            private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

            // Since I screwed up nicely in my documentation, I'm going to add errors
            // informing the user if they've set one of the wrong parameters
            if (private_nh.hasParam("acc_limit_x"))
                ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

            if (private_nh.hasParam("acc_limit_y"))
                ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

            if (private_nh.hasParam("acc_limit_th"))
                ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

            // Assuming this planner is being run within the navigation stack, we can
            // just do an upward search for the frequency at which its being run. This
            // also allows the frequency to be overwritten locally.
            std::string controller_frequency_param_name;
            if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
                sim_period_ = 0.05;
            else
            {
                double controller_frequency = 0;
                private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
                if (controller_frequency > 0)
                    sim_period_ = 1.0 / controller_frequency;
                else
                {
                    ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                    sim_period_ = 0.05;
                }
            }
            ROS_INFO("Sim period is set to %.2f", sim_period_);

            private_nh.param("sim_time", sim_time, 1.0);
            private_nh.param("sim_granularity", sim_granularity, 0.025);
            private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
            private_nh.param("vx_samples", vx_samples, 3);
            private_nh.param("vtheta_samples", vtheta_samples, 20);

            path_distance_bias = nav_core::loadParameterWithDeprecation(private_nh,
                                                                        "path_distance_bias",
                                                                        "pdist_scale",
                                                                        0.6);
            goal_distance_bias = nav_core::loadParameterWithDeprecation(private_nh,
                                                                        "goal_distance_bias",
                                                                        "gdist_scale",
                                                                        0.6);
            // values of the deprecated params need to be applied to the current params, as defaults
            // of defined for dynamic reconfigure will override them otherwise.
            if (private_nh.hasParam("pdist_scale") & !private_nh.hasParam("path_distance_bias"))
            {
                private_nh.setParam("path_distance_bias", path_distance_bias);
            }
            if (private_nh.hasParam("gdist_scale") & !private_nh.hasParam("goal_distance_bias"))
            {
                private_nh.setParam("goal_distance_bias", goal_distance_bias);
            }

            private_nh.param("occdist_scale", occdist_scale, 0.01);

            bool meter_scoring;
            if (!private_nh.hasParam("meter_scoring"))
            {
                ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
            }
            else
            {
                private_nh.param("meter_scoring", meter_scoring, false);

                if (meter_scoring)
                {
                    // if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
                    double resolution = costmap_->getResolution();
                    goal_distance_bias *= resolution;
                    path_distance_bias *= resolution;
                }
                else
                {
                    ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
                }
            }

            private_nh.param("heading_lookahead", heading_lookahead, 0.325);
            private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
            private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
            private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
            private_nh.param("holonomic_robot", holonomic_robot, true);
            private_nh.param("max_vel_x", max_vel_x, 0.5);
            private_nh.param("min_vel_x", min_vel_x, 0.1);

            double max_rotational_vel;
            private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
            max_vel_th_ = max_rotational_vel;
            min_vel_th_ = -1.0 * max_rotational_vel;

            min_in_place_vel_th_ = nav_core::loadParameterWithDeprecation(private_nh,
                                                                          "min_in_place_vel_theta",
                                                                          "min_in_place_rotational_vel", 0.4);
            reached_goal_ = false;
            backup_vel = -0.1;
            if (private_nh.getParam("backup_vel", backup_vel))
                ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

            // if both backup_vel and escape_vel are set... we'll use escape_vel
            private_nh.getParam("escape_vel", backup_vel);

            if (backup_vel >= 0.0)
                ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

            private_nh.param("world_model", world_model_type, std::string("costmap"));
            private_nh.param("dwa", dwa, true);
            private_nh.param("heading_scoring", heading_scoring, false);
            private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

            simple_attractor = false;

            // parameters for using the freespace controller
            double min_pt_separation, max_obstacle_height, grid_resolution;
            private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
            private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
            private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
            private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

            ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");

            footprint_spec_ = costmap_ros_->getRobotFootprint();
            initialized_ = true;

            // dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
            // dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
            // dsrv_->setCallback(cb);

            apf_ = new potbot_lib::ArtificialPotentialFieldROS("potbot/potential_field");
            apf_planner_ = new potbot_lib::path_planner::APFPathPlannerROS("potbot/path_planner", apf_);
            robot_controller_ = new potbot_lib::controller::DiffDriveControllerROS("potbot/controller");
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing");
        }
    }

    bool PotbotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // when we get a new plan, we also want to clear any latch we may have on goal tolerances
        xy_tolerance_latch_ = false;
        // reset the at goal flag
        reached_goal_ = false;
        return true;
    }

    bool PotbotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped global_pose;
        if (!costmap_ros_->getRobotPose(global_pose))
        {
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        // get the global plan in our frame
        if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }

        // now we'll prune the plan based on the position of the robot
        if (prune_plan_)
            prunePlan(global_pose, transformed_plan, global_plan_);

        // if the global plan passed in is empty... we won't do anything
        if (transformed_plan.empty())
            return false;

        const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();

        apf_->initPotentialField(costmap_ros_);

        apf_->setGoal(transformed_plan.back());

        apf_->createPotentialField();

        double init_yaw = tf2::getYaw(global_pose.pose.orientation);

        ROS_DEBUG("status: create path");
        apf_planner_->createPath(init_yaw);

        ROS_DEBUG("status: interpolate");
        apf_planner_->bezier();

        apf_planner_->publishPath();

        nav_msgs::Path path_msg_interpolated;
        apf_planner_->getPath(path_msg_interpolated);

        apf_->publishPotentialField();

        robot_controller_->setMsg(global_pose);
        robot_controller_->deltatime = 1.0 / 30.0;

        robot_controller_->setTarget(global_plan_.back().pose);
        reached_goal_ = robot_controller_->reachedTarget();
        nav_msgs::Odometry sim_pose;
        if (!reached_goal_)
        {
            potbot_lib::Point p;
            p.x = goal_point.pose.position.x;
            p.y = goal_point.pose.position.y;
            if (robot_controller_->getDistance(p) < 0.6)
            {
                robot_controller_->pidControl();
                robot_controller_->toMsg(sim_pose);
            }
            else
            {

                potbot_lib::controller::DWAControllerROS dwa;
                dwa.setMsg(global_pose);
                dwa.setDwaTargetPath(path_msg_interpolated);
                robot_controller_->initPID();
                dwa.calculateCommand();
                dwa.toMsg(sim_pose);


                // robot_controller_->set_target_path(path_msg_interpolated);
                // robot_controller_->initPID();
                // robot_controller_->normalized_pure_pursuit();
                // // robot_controller_.pure_pursuit();
                // robot_controller_->publishLookahead();
                
                // robot_controller_->to_msg(sim_pose);
                
            }
            cmd_vel = sim_pose.twist.twist;
        }
        else
        {
            ROS_INFO("reached target");
        }

        // //publish information to the visualizer
        publishPlan(transformed_plan, g_plan_pub_);
        // publishPlan(local_plan, l_plan_pub_);
        return true;
    }

    bool PotbotLocalPlanner::isGoalReached()
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        // return flag set in controller
        return reached_goal_;
    }

};
