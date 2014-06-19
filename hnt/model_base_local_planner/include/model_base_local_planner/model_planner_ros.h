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
*   * Neither the name of University of Nevada, Reno nor the names of its
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
#ifndef MODEL_TRAJECTORY_PLANNER_ROS_H_
#define MODEL_TRAJECTORY_PLANNER_ROS_H_

#include <model_base_local_planner/model_planner.h>
#include <model_base_local_planner/trajectory_planner_ros.h>

namespace model_base_local_planner {
  /**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class ModelTrajectoryPlannerROS : public base_local_planner::TrajectoryPlannerROS {
    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      ModelTrajectoryPlannerROS() : base_local_planner::TrajectoryPlannerROS() {}

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      ModelTrajectoryPlannerROS(std::string name,
                           tf::TransformListener* tf,
                           costmap_2d::Costmap2DROS* costmap_ros) : base_local_planner::TrajectoryPlannerROS(name,tf,costmap_ros) {}

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);
     
      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    protected:
  };
};
#endif
