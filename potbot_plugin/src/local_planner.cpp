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

    PotbotLocalPlanner::PotbotLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), controller_loader_("potbot_base", "potbot_base::Controller"), planner_loader_("potbot_base", "potbot_base::PathPlanner") {}

    PotbotLocalPlanner::PotbotLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), controller_loader_("potbot_base", "potbot_base::Controller"), planner_loader_("potbot_base", "potbot_base::PathPlanner")
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

            path_planner_thread_ = new boost::thread(boost::bind(&PotbotLocalPlanner::createPathThread, this));

            // dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
            // dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
            // dsrv_->setCallback(cb);
            
            // robot_controller_ = new potbot_lib::controller::DiffDriveControllerROS("potbot/controller");
            std::string plugin_name = "potbot_nav/OPF";
            private_nh.getParam("controller_name", plugin_name);
            try
            {
                controller_ = controller_loader_.createInstance(plugin_name);
                controller_->initialize(name + "/controller", tf);
                ROS_INFO("\t%s initialized", plugin_name.c_str());
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("failed to load plugin. Error: %s", ex.what());
            }

            std::string planner_plugin_name = "potbot_nav/APF";
            private_nh.getParam("path_planner_name", planner_plugin_name);
            try
            {
                planner_ = planner_loader_.createInstance(planner_plugin_name);
                planner_->initialize(name + "/path_planner", tf);
                ROS_INFO("\t%s initialized", planner_plugin_name.c_str());
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR("failed to load plugin. Error: %s", ex.what());
            }
            
            std::string recover_plugin_name = "potbot_nav/PID";
            recover_ = controller_loader_.createInstance(recover_plugin_name);
            recover_->initialize(name + "/recover", tf);
            ROS_INFO("\t%s initialized", recover_plugin_name.c_str());

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
        // ROS_INFO("setPlan");
        // potbot_lib::utility::print_pose(global_plan_.back().pose);
        recover_->setTargetPose(global_plan_.back());

        // when we get a new plan, we also want to clear any latch we may have on goal tolerances
        xy_tolerance_latch_ = false;
        // reset the at goal flag
        reached_goal_ = false;
        return true;
    }

    void costmapToObstacles(const costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::Point>& obstacle_vec)
    {
        obstacle_vec.clear();
        unsigned char* costs = costmap->getCharMap();
        unsigned int map_size = costmap->getSizeInCellsX()*costmap->getSizeInCellsY();
        for (unsigned int i = 0; i < map_size; i++)
        {
            int cost = costs[i];
            if (cost == 254)
            {
                unsigned int xi,yi;
                costmap->indexToCells(i,xi,yi);
                double x,y;
                costmap->mapToWorld(xi,yi,x,y);
                obstacle_vec.push_back(potbot_lib::utility::get_point(x, y));
            }
        }
    }

    void PotbotLocalPlanner::createPathThread()
    {
        while (ros::ok())
        {
            if (!initialized_)
            {
                ROS_WARN("not initialized");
                ros::Duration(1).sleep();
                continue;
            }

            std::vector<geometry_msgs::PoseStamped> local_plan;
            geometry_msgs::PoseStamped global_pose;
            if (!costmap_ros_->getRobotPose(global_pose))
            {
                // return false;
                continue;
            }

            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            // get the global plan in our frame
            if (!global_plan_.empty() && !transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
            {
                ROS_WARN("Could not transform the global plan to the frame of the controller");
                // return false;
                continue;
            }

            // now we'll prune the plan based on the position of the robot
            if (prune_plan_)
                prunePlan(global_pose, transformed_plan, global_plan_);

            // if the global plan passed in is empty... we won't do anything
            if (transformed_plan.empty())
                // return false;
                continue;

            const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();

            // apf_->initPotentialField(costmap_ros_);
            // apf_->setGoal(transformed_plan.back());
            // apf_->createPotentialField();
            // apf_->publishPotentialField();

            // double init_yaw = tf2::getYaw(global_pose.pose.orientation);
            // ROS_DEBUG("status: create path");
            // apf_planner_->createPath(init_yaw);
            // ROS_DEBUG("status: interpolate");
            // apf_planner_->bezier();
            // apf_planner_->publishPath();

            nav_msgs::Odometry nav_robot;
            nav_robot.header = global_pose.header;
            nav_robot.pose.pose = global_pose.pose;
            planner_->setRobot(nav_robot);
            planner_->setTargetPose(transformed_plan.back());
            std::vector<geometry_msgs::Point> obs;
            costmapToObstacles(costmap_, obs);
            // ROS_INFO("obstacles size: %d", obs.size());
            planner_->clearObstacles();
            planner_->setObstacles(obs);
            planner_->planPath();

            nav_msgs::Path path_msg;
            planner_->getPath(path_msg.poses);
            path_msg.header.frame_id = global_frame_;
            path_msg.header.stamp = ros::Time::now();
            l_plan_pub_.publish(path_msg);
            path_msg.poses = transformed_plan;
            g_plan_pub_.publish(path_msg);
        }
    }

    bool PotbotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    { 
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        geometry_msgs::PoseStamped global_pose;
        if (!costmap_ros_->getRobotPose(global_pose))
        {
            ROS_INFO("return code: false global_pose");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        // get the global plan in our frame
        if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }

        double distance_to_goal = potbot_lib::utility::get_distance(global_pose.pose, transformed_plan.back().pose);

        if (distance_to_goal < 0.1)
        {
            reached_goal_ = true;
            ROS_INFO("reached target 0");
        }
        else if (distance_to_goal < 0.3)
        {
            nav_msgs::Odometry sim_pose;
            sim_pose.pose.pose = global_pose.pose;
            recover_->setRobot(sim_pose);
            recover_->calculateCommand(cmd_vel);
        }
        else
        {
            nav_msgs::Path path_msg_interpolated;
            planner_->getPath(path_msg_interpolated.poses);
            if (path_msg_interpolated.poses.size() > 1)
            {
                controller_->setTargetPath(path_msg_interpolated.poses);
                // controller_->setTargetPath(global_plan_);
                // controller_->setTargetPose(global_plan_.back());
                
            }
            nav_msgs::Odometry sim_pose;
            sim_pose.pose.pose = global_pose.pose;
            controller_->setRobot(sim_pose);
            controller_->calculateCommand(cmd_vel);
            // ROS_INFO_STREAM(reached_goal_);
            // reached_goal_ = controller_->reachedTarget();
            // ROS_INFO_STREAM(reached_goal_);
            // ROS_INFO("%d, %f, %f",path_msg_interpolated.poses.size(), cmd_vel.linear.x, cmd_vel.angular.z);
        }

        //publish information to the visualizer
        // publishPlan(transformed_plan, g_plan_pub_);
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
