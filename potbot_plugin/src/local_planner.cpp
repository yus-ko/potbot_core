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

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(potbot_nav::PotbotLocalPlanner, nav_core::BaseLocalPlanner)

namespace potbot_nav {

  PotbotLocalPlanner::PotbotLocalPlanner() :
      costmap_ros_(NULL), tf_(NULL), initialized_(false){}

  PotbotLocalPlanner::PotbotLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      costmap_ros_(NULL), tf_(NULL), initialized_(false){

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void PotbotLocalPlanner::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

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

      //initialize the copy of the costmap the controller will use
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

      //Since I screwed up nicely in my documentation, I'm going to add errors
      //informing the user if they've set one of the wrong parameters
      if(private_nh.hasParam("acc_limit_x"))
        ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_y"))
        ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_th"))
        ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      //Assuming this planner is being run within the navigation stack, we can
      //just do an upward search for the frequency at which its being run. This
      //also allows the frequency to be overwritten locally.
      std::string controller_frequency_param_name;
      if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
      else
      {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0)
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
      if ( ! private_nh.hasParam("meter_scoring")) {
        ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
      } else {
        private_nh.param("meter_scoring", meter_scoring, false);

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          goal_distance_bias *= resolution;
          path_distance_bias *= resolution;
        } else {
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
      if(private_nh.getParam("backup_vel", backup_vel))
        ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

      //if both backup_vel and escape_vel are set... we'll use escape_vel
      private_nh.getParam("escape_vel", backup_vel);

      if(backup_vel >= 0.0)
        ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      private_nh.param("world_model", world_model_type, std::string("costmap"));
      private_nh.param("dwa", dwa, true);
      private_nh.param("heading_scoring", heading_scoring, false);
      private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
      // world_model_ = new CostmapModel(*costmap_);
      // std::vector<double> y_vels = loadYVels(private_nh);

      footprint_spec_ = costmap_ros_->getRobotFootprint();

      // tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
      //     acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, path_distance_bias,
      //     goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
      //     max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
      //     dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      // map_viz_.initialize(name,
      //                     global_frame_,
      //                     [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost){
      //                         return tc_->getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
      //                     });
      initialized_ = true;

      // dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
      // dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
      // dsrv_->setCallback(cb);

      robot_controller_.set_gain(	    1.0,
                                      0.1,
                                      0.001);

      robot_controller_.set_margin(	yaw_goal_tolerance_,
                                    xy_goal_tolerance_);

      robot_controller_.set_limit(	max_vel_x,
                                    max_vel_th_);
      
      robot_controller_.set_distance_to_lookahead_point(0.4);

    } else {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  bool PotbotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

  bool PotbotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // ROS_INFO("compute velocity");
    // cmd_vel.linear.x=0.2;
    // return true;

    std::vector<geometry_msgs::PoseStamped> local_plan;
    geometry_msgs::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);

    // geometry_msgs::PoseStamped drive_cmds;
    // drive_cmds.header.frame_id = robot_base_frame_;

    // geometry_msgs::PoseStamped robot_vel;
    // odom_helper_.getRobotVel(robot_vel);

    // /* For timing uncomment
    // struct timeval start, end;
    // double start_t, end_t, t_diff;
    // gettimeofday(&start, NULL);
    // */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    // const geometry_msgs::PoseStamped& goal_point = transformed_plan.back();
    // //we assume the global goal is the last point in the global plan
    // const double goal_x = goal_point.pose.position.x;
    // const double goal_y = goal_point.pose.position.y;

    // const double yaw = tf2::getYaw(goal_point.pose.orientation);

    // double goal_th = yaw;

    // //check to see if we've reached the goal position
    // if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

    //   //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //   //just rotate in place
    //   if (latch_xy_goal_tolerance_) {
    //     xy_tolerance_latch_ = true;
    //   }

    //   double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
    //   //check to see if the goal orientation has been reached
    //   if (fabs(angle) <= yaw_goal_tolerance_) {
    //     //set the velocity command to zero
    //     cmd_vel.linear.x = 0.0;
    //     cmd_vel.linear.y = 0.0;
    //     cmd_vel.angular.z = 0.0;
    //     rotating_to_goal_ = false;
    //     xy_tolerance_latch_ = false;
    //     reached_goal_ = true;
    //   } else {
    //     //we need to call the next two lines to make sure that the trajectory
    //     //planner updates its path distance and goal distance grids
    //     tc_->updatePlan(transformed_plan);
    //     Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
    //     map_viz_.publishCostCloud(costmap_);

    //     //copy over the odometry information
    //     nav_msgs::Odometry base_odom;
    //     odom_helper_.getOdom(base_odom);

    //     //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    //     if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
    //       if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
    //         return false;
    //       }
    //     }
    //     //if we're stopped... then we want to rotate to goal
    //     else{
    //       //set this so that we know its OK to be moving
    //       rotating_to_goal_ = true;
    //       if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
    //         return false;
    //       }
    //     }
    //   }

      // //publish an empty plan because we've reached our goal position
      // publishPlan(transformed_plan, g_plan_pub_);
    //   publishPlan(local_plan, l_plan_pub_);

    //   //we don't actually want to run the controller when we're just rotating to goal
    //   return true;
    // }

    nav_msgs::Path path_msg;
    path_msg.poses = transformed_plan;
    robot_controller_.set_target_path(path_msg);
    robot_controller_.set_msg(global_pose);
    robot_controller_.deltatime = 1.0/30.0;

    robot_controller_.normalized_pure_pursuit();
    // robot_controller_.pure_pursuit();

    nav_msgs::Odometry sim_pose;
    robot_controller_.to_msg(sim_pose);
    
    // visualization_msgs::Marker lookahead_msg;
    // robot_controller_.get_lookahead(lookahead_msg);
    // lookahead_msg.header = odom_.header;

    cmd_vel = sim_pose.twist.twist;

    // tc_->updatePlan(transformed_plan);

    // //compute what trajectory to drive along
    // Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

    // map_viz_.publishCostCloud(costmap_);
    // /* For timing uncomment
    // gettimeofday(&end, NULL);
    // start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    // end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    // t_diff = end_t - start_t;
    // ROS_INFO("Cycle time: %.9f", t_diff);
    // */

    // //pass along drive commands
    // cmd_vel.linear.x = drive_cmds.pose.position.x;
    // cmd_vel.linear.y = drive_cmds.pose.position.y;
    // cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    // //if we cannot move... tell someone
    // if (path.cost_ < 0) {
    //   ROS_DEBUG_NAMED("trajectory_planner_ros",
    //       "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
    //   local_plan.clear();
    //   publishPlan(transformed_plan, g_plan_pub_);
    //   publishPlan(local_plan, l_plan_pub_);
    //   return false;
    // }

    // ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
    //     cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // // Fill out the local plan
    // for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
    //   double p_x, p_y, p_th;
    //   path.getPoint(i, p_x, p_y, p_th);
    //   geometry_msgs::PoseStamped pose;
    //   pose.header.frame_id = global_frame_;
    //   pose.header.stamp = ros::Time::now();
    //   pose.pose.position.x = p_x;
    //   pose.pose.position.y = p_y;
    //   pose.pose.position.z = 0.0;
    //   tf2::Quaternion q;
    //   q.setRPY(0, 0, p_th);
    //   tf2::convert(q, pose.pose.orientation);
    //   local_plan.push_back(pose);
    // }

    // //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_);
    // publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  bool PotbotLocalPlanner::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_; 
  }

};
