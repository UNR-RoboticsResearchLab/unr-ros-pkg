//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef OBJECT_TRACKER_PARAMETERS_H
#define OBJECT_TRACKER_PARAMETERS_H

#include <string>
#include <map>
#include <std_msgs/ColorRGBA.h>
#include <ros/service_client.h>

namespace hector_object_tracker {

  extern std::map<std::string, bool>   _project_objects;
  extern std::map<std::string, double> _default_distance;
  extern std::map<std::string, double> _distance_variance;
  extern std::map<std::string, double> _angle_variance;
  extern std::map<std::string, double> _min_height;
  extern std::map<std::string, double> _max_height;
  extern std::map<std::string, double> _pending_support;
  extern std::map<std::string, double> _pending_time;
  extern std::map<std::string, double> _active_support;
  extern std::map<std::string, double> _active_time;
  extern std::map<std::string, double> _inactive_support;
  extern std::map<std::string, double> _inactive_time;
  extern std::map<std::string, std_msgs::ColorRGBA> _marker_color;
  extern std::map<std::string, ros::ServiceClient> _distance_to_obstacle_service;

  namespace Parameters {
    void load(const std::string& class_id = std::string());
  }

  template <typename T> static inline T& parameter(std::map<std::string, T>& p, const std::string& class_id = std::string()) {
    if (p.count(class_id)) return p.at(class_id);
    return p[std::string()];
  }

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_PARAMETERS_H
