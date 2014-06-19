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

#include <model_base_local_planner/model_planner_ros.h>
#include <model_base_local_planner/model_planner.h>

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(model_base_local_planner::ModelTrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace model_base_local_planner {
  /*
  ModelTrajectoryPlannerROS::ModelTrajectoryPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }
*/
  void ModelTrajectoryPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
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
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
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
      private_nh.param("acc_lim_th", acc_lim_theta_, 3.2);

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

      private_nh.param("path_distance_bias", pdist_scale, 0.6);
      private_nh.param("goal_distance_bias", gdist_scale, 0.8);
      private_nh.param("occdist_scale", occdist_scale, 0.01);

      bool meter_scoring;
      if ( ! private_nh.hasParam("meter_scoring")) {
        ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settins robust against changes of costmap resolution.");
      } else {
        private_nh.param("meter_scoring", meter_scoring, false);

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          gdist_scale *= resolution;
          pdist_scale *= resolution;
          occdist_scale *= resolution;
        } else {
          ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
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
      private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.4);

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
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      std::vector<double> y_vels = loadYVels(private_nh);

      footprint_spec_ = costmap_ros_->getRobotFootprint();

      tc_ = new ModelTrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      map_viz_.initialize(name, global_frame_, boost::bind(&ModelTrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<base_local_planner::BaseLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<base_local_planner::BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&ModelTrajectoryPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

    } else {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  bool ModelTrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if (xy_tolerance_latch_ || (base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
      } else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan(transformed_plan);
        base_local_planner::Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }

      //publish an empty plan because we've reached our goal position
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
      base_local_planner::publishPlan(local_plan, l_plan_pub_);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    base_local_planner::Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

    map_viz_.publishCostCloud(costmap_);
    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    if (path.cost_ < 0) {
      ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
      base_local_planner::publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      tf::Stamped<tf::Pose> p =
          tf::Stamped<tf::Pose>(tf::Pose(
              tf::createQuaternionFromYaw(p_th),
              tf::Point(p_x, p_y, 0.0)),
              ros::Time::now(),
              global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
    base_local_planner::publishPlan(local_plan, l_plan_pub_);
    return true;
  }


};
