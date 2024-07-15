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

    PotbotLocalPlanner::PotbotLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), loader_("potbot_base", "potbot_base::Controller") {}

    PotbotLocalPlanner::PotbotLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), loader_("potbot_base", "potbot_base::Controller")
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

            // initialize the copy of the costmap the controller will use
            costmap_ = costmap_ros_->getCostmap();

            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();
            private_nh.param("prune_plan", prune_plan_, true);

            reached_goal_ = false;
            

            // dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
            // dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
            // dsrv_->setCallback(cb);

            apf_ = new potbot_lib::ArtificialPotentialFieldROS("potbot/potential_field");
            apf_planner_ = new potbot_lib::path_planner::APFPathPlannerROS("potbot/path_planner", apf_);
            // robot_controller_ = new potbot_lib::controller::DiffDriveControllerROS("potbot/controller");
            std::string plugin_name = "potbot_nav/OPF";
            private_nh.getParam("controller_name", plugin_name);
            try
            {
                controller_ = loader_.createInstance(plugin_name);
                controller_->initialize(name + "/controller", tf);
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("failed to load plugin. Error: %s", ex.what());
            }

            initialized_ = true;
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
        apf_->publishPotentialField();

        double init_yaw = tf2::getYaw(global_pose.pose.orientation);
        ROS_DEBUG("status: create path");
        apf_planner_->createPath(init_yaw);
        ROS_DEBUG("status: interpolate");
        apf_planner_->bezier();
        apf_planner_->publishPath();

        nav_msgs::Path path_msg_interpolated;
        apf_planner_->getPath(path_msg_interpolated);
        if (!reached_goal_)
        {
            controller_->setTargetPath(path_msg_interpolated.poses);
            // controller_->setTargetPose(global_plan_.back());
            nav_msgs::Odometry sim_pose;
            sim_pose.pose.pose = global_pose.pose;
            controller_->setRobot(sim_pose);
            controller_->calculateCommand(cmd_vel);
            reached_goal_ = controller_->reachedTarget();
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
