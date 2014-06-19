/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, David Feil-Seifer, Willow Garage.
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
*   * Neither the name of the University of Nevada, Reno, Willow
*     Garage nor the names of its contributors may be used to endorse
*     or promote products derived from this software without specific
*     prior written permission.
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
* Author: Eitan Marder-Eppstein, David Feil-Seifer
*********************************************************************/

#include <model_base_local_planner/model_planner.h>

#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>



#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>

using namespace std;
using namespace costmap_2d;

namespace model_base_local_planner {

  /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
   
  void ModelTrajectoryPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      base_local_planner::Trajectory& traj) {

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = ::hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    if(!heading_scoring_) {
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp; 
    traj.yv_ = vy_samp; 
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
        //TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
        //it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to 
        //come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
        //but safe.

        /*
        double max_vel_x, max_vel_y, max_vel_th;
        //we want to compute the max allowable speeds to be able to stop
        //to be safe... we'll make sure we can stop some time before we actually hit
        getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

        //check if we can stop in time
        if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
          ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
          //if we can stop... we'll just break out of the loop here.. no point in checking future points
          break;
        }
        else{
          traj.cost_ = -1.0;
          return;
        }
        */
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) * 
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) + 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) * 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {

        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if (!heading_scoring_) {
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
    } else {
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    }
    traj.cost_ = cost;
  }


};


