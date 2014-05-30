/*
 * point_map
 * Copyright (c) 2014, David Feil-Seifer, University of Nevada, Reno
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <point_map/PointMap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <stdlib.h>


double resolution = .01;
double map_size = 7.0;
int map_width = (int) (map_size / resolution);
int map_height = (int) (map_size / resolution);


template<typename _T>
struct Point2D
{
  _T x;
  _T y;
};

struct Segment
{
  bool close;
  std::vector<Point2D<float> > points;
  
  void drawSegment(std::vector<signed char, std::allocator<signed char> > & mapData)
  {
    // for each line...
    
    unsigned int num = points.size() - (close ? 0 : 1);

    for( unsigned int i = 0; i < num; i++ )
    {
      // is the line more horizontal or vertical?
      int idx1 = i, idx2 = close ? ( i+1 ) % num : ( i+1 );
      double run  = points[idx2].x - points[idx1].x;
      double rise = points[idx2].y - points[idx1].y;

      double m = rise / run;
      int u0 = (int) (points[idx1].x / resolution ) + map_width / 2;
      int un = (int) (points[idx2].x / resolution ) + map_height / 2;
      int v0 = (int) (points[idx1].y / resolution ) + map_width / 2;
      int vn = (int) (points[idx2].y / resolution ) + map_height / 2;

      if( fabs( run ) > fabs( rise ) )
      {

        for( int j = u0; j != un; j += run / fabs(run) )
        {
          int u = j;
          int v = v0 + (u-u0)*m;
          mapData[u + (v-1)*map_width] = 100;
          mapData[u + v*map_width] = 100;
          mapData[u + (v+1)*map_width] = 100;
        }
      }
      else
      {     
        for( int j = v0; j != vn ; j+= rise / fabs( rise) )
        {
          int v = j;
          int u = u0 + (v-v0)/m;
          mapData[u-1 + v*map_width] = 100;
          mapData[u+1 + v*map_width] = 100;
          mapData[u + v*map_width] = 100;
        }

      }
    }
  }
};

std::vector<Segment> mSegments;
std::vector<Point2D<float>* > allPoints;

namespace YAML {
  template<>
  struct convert<Point2D<float> > {
    static bool decode(const Node& node, Point2D<float>&p) {
      if(!node.IsSequence() || node.size() != 2)
      {
        printf( "node is wrong\n" );
        return false;
      }

      p.x = node[0].as<float>();
      p.y = node[1].as<float>();
      printf("%f, %f\n", p.x, p.y );
      return true;
    }
  };

  template<>
  struct convert<Segment> {
    static bool decode(const Node & node, Segment & s)
    {
      s.close = node["close"].as<bool>();
      const YAML::Node & points = node["points"];
      s.points.resize( points.size() );
      printf( "points.size: %d\n", points.size() );
      for( unsigned int i = 0; i < points.size(); i ++ )
      {
        s.points[i] = points[i].as<Point2D<float> >();
    
        //record point to be published w/ map info later
        allPoints.push_back( & (s.points[i]) );
      }
      return true;
    }
  };
}

bool map(point_map::PointMap::Request  &req,
         point_map::PointMap::Response &res )
{

  res.num_points = allPoints.size(); //num lines

  ROS_INFO( "request for map made... \n" );
  ROS_INFO( "responding with map of size: [%d]\n", (int) res.num_points );

  res.x.resize(res.num_points);
  res.y.resize(res.num_points);
  
  for( int i = 0; i < res.num_points; i++ )
  {
    ROS_INFO( "(%0.2f,%0.2f) ", allPoints[i]->x,allPoints[i]->y );
    res.x[i] = allPoints[i]->x;
    res.y[i] = allPoints[i]->y;
  }

  return true;
}

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"map_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("map_server", map );
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",0);
  
  // make map occupancy grid...

  nav_msgs::OccupancyGrid map_msg;
  map_msg.info.map_load_time = ros::Time::now();
  map_msg.info.resolution = resolution;
  map_msg.info.width = map_width;
  map_msg.info.height = map_height;
  geometry_msgs::Pose pose;
  pose.position.x = -map_size/2.;
  pose.position.y = -map_size/2.;
  map_msg.info.origin = pose;
  
  int ncells = map_width*map_height;
  map_msg.data.resize(ncells);
  for( int i = 0; i < ncells; i++ )
  {
    map_msg.data[i] = 0;
  }
  
  //read file
  
  if(argc < 2)
  {
    ROS_WARN("You must provide a map file to read from");
    return 1;
  }

  YAML::Node doc = YAML::LoadFile(argv[1]);
  printf( "num segments: %d\n", doc.size() );
  mSegments.resize( doc.size() );
  

  for( unsigned int i = 0; i < doc.size(); i ++ )
  {
    mSegments[i] = doc[i].as<Segment>();
  }
  
  // for each segment
  
  for( unsigned int i = 0; i < mSegments.size(); i ++ )
  {
    mSegments[i].drawSegment(map_msg.data);
  }

  map_msg.header.frame_id = "/map";
  while( ros::ok() )
  {
    map_msg.header.stamp = ros::Time::now();
    map_pub.publish( map_msg );

    printf( "." ); fflush(stdout );
    ros::spinOnce();
    ros::Duration(5.0).sleep();
  }

  return 0;
}
