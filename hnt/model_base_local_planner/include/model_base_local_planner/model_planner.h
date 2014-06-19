/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, David Feil-Seifer
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
*   * Neither the name of the University of Nevada, Reno  nor the names of its
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
* Author: David Feil-Seifer
*********************************************************************/
#ifndef MODEL_PLANNER_H_
#define MODEL_PLANNER_H_

#include <model_base_local_planner/trajectory_planner.h>

namespace model_base_local_planner {
  /**
   * @class ModelPlanner
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world and an auxillary model. 
   */
  class ModelTrajectoryPlanner : public base_local_planner::TrajectoryPlanner {

    public:
      /**
       * @brief  Constructs a trajectory controller
       * @param world_model The WorldModel the trajectory controller uses to check for collisions 
       * @param costmap A reference to the Costmap the controller should use
       * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       * @param inscribed_radius The radius of the inscribed circle of the robot
       * @param circumscribed_radius The radius of the circumscribed circle of the robot
       * @param acc_lim_x The acceleration limit of the robot in the x direction
       * @param acc_lim_y The acceleration limit of the robot in the y direction
       * @param acc_lim_theta The acceleration limit of the robot in the theta direction
       * @param sim_time The number of seconds to "roll-out" each trajectory
       * @param sim_granularity The distance between simulation points should be small enough that the robot doesn't hit things
       * @param vx_samples The number of trajectories to sample in the x dimension
       * @param vtheta_samples The number of trajectories to sample in the theta dimension
       * @param pdist_scale A scaling factor for how close the robot should stay to the path
       * @param gdist_scale A scaling factor for how aggresively the robot should pursue a local goal
       * @param occdist_scale A scaling factor for how much the robot should prefer to stay away from obstacles
       * @param heading_lookahead How far the robot should look ahead of itself when differentiating between different rotational velocities
       * @param oscillation_reset_dist The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
       * @param escape_reset_dist The distance the robot must travel before it can exit escape mode
       * @param escape_reset_theta The distance the robot must rotate before it can exit escape mode
       * @param holonomic_robot Set this to true if the robot being controlled can take y velocities and false otherwise
       * @param max_vel_x The maximum x velocity the controller will explore
       * @param min_vel_x The minimum x velocity the controller will explore
       * @param max_vel_th The maximum rotational velocity the controller will explore
       * @param min_vel_th The minimum rotational velocity the controller will explore
       * @param min_in_place_vel_th The absolute value of the minimum in-place rotational velocity the controller will explore
       * @param backup_vel The velocity to use while backing up
       * @param dwa Set this to true to use the Dynamic Window Approach, false to use acceleration limits
       * @param heading_scoring Set this to true to score trajectories based on the robot's heading after 1 timestep
       * @param heading_scoring_timestep How far to look ahead in time when we score heading based trajectories
       * @param meter_scoring adapt parameters to costmap resolution
       * @param simple_attractor Set this to true to allow simple attraction to a goal point instead of intelligent cost propagation
       * @param y_vels A vector of the y velocities the controller will explore
       * @param angular_sim_granularity The distance between simulation points for angular velocity should be small enough that the robot doesn't hit things
       */
      ModelTrajectoryPlanner(base_local_planner::WorldModel& world_model, 
          const costmap_2d::Costmap2D& costmap, 
          std::vector<geometry_msgs::Point> footprint_spec,
          double acc_lim_x, double acc_lim_y, double acc_lim_theta,
          double sim_time, double sim_granularity, 
          int vx_samples, int vtheta_samples,
          double pdist_scale, double gdist_scale, double occdist_scale,
          double heading_lookahead, double oscillation_reset_dist, 
          double escape_reset_dist, double escape_reset_theta,
          bool holonomic_robot,
          double max_vel_x, double min_vel_x, 
          double max_vel_th, double min_vel_th, double min_in_place_vel_th,
          double backup_vel,
          bool dwa, bool heading_scoring, double heading_scoring_timestep,
          bool meter_scoring,
          bool simple_attractor,
          std::vector<double> y_vels,
          double stop_time_buffer,
          double sim_period, double angular_sim_granularity) : TrajectoryPlanner( world_model, costmap, footprint_spec, acc_lim_x, acc_lim_y, acc_lim_theta,sim_time,sim_granularity, vx_samples, vtheta_samples, pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot, max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th, backup_vel, dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period, angular_sim_granularity) 
          {
          }

      void generateTrajectory(double x, double y, double theta, double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y, double acc_theta, double impossible_cost, base_local_planner::Trajectory& traj);

  };
};

#endif
