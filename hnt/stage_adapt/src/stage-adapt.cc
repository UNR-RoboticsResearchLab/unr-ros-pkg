#include <iostream>
#include <string.h>
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

tf::Transform ident, bandit_offset;
std::string robot_name_, child_name_, parent_name_;
    
tf::TransformBroadcaster* tb;
tf::TransformListener* tl;
ros::Publisher cmdvel_pub, robotodom_pub;

ros::Time stamp;

void cmdvel_cb( const geometry_msgs::TwistConstPtr& msg )
{
  geometry_msgs::Twist newmsg = *msg;
  cmdvel_pub.publish( newmsg );
}

void odom_cb( const nav_msgs::OdometryConstPtr& msg )
{
	nav_msgs::Odometry newmsg = *msg;
	newmsg.header.frame_id = "/robot/odom";
	robotodom_pub.publish(newmsg);
}

void robot_cb( const nav_msgs::OdometryConstPtr& msg )
{

  geometry_msgs::PoseStamped p;
  p.header.stamp = msg->header.stamp;
  p.pose = msg->pose.pose;

  tf::Transform pt;
  tf::poseMsgToTF(p.pose,pt);
  tf::StampedTransform tx( pt,p.header.stamp,"/ovh", "/robot/base_link");
  tb->sendTransform(tx); 
	nav_msgs::Odometry newmsg;
  geometry_msgs::PoseStamped identpose, odompose;


  identpose.header.frame_id = std::string(robot_name_+"/odom");
  identpose.header.stamp = msg->header.stamp;

  identpose.pose.position.x = 0;
  identpose.pose.position.y = 0;
  identpose.pose.position.z = 0;
  
  identpose.pose.orientation.x = 0;
  identpose.pose.orientation.y = 0;
  identpose.pose.orientation.z = 0;
  identpose.pose.orientation.w = 1;

  try {
		tf::StampedTransform trans;
		tl->lookupTransform( robot_name_ + "/base_link", robot_name_+"/odom", ros::Time(0), trans );
		trans.frame_id_ = "/robot/base_link";
		trans.child_frame_id_ = "/robot/odom";
		geometry_msgs::TransformStamped ps;
		tf::transformStampedTFToMsg( trans, ps );

		newmsg = *msg;
		newmsg.pose.pose.position.x = ps.transform.translation.x;
		newmsg.pose.pose.position.y = ps.transform.translation.y;
		newmsg.pose.pose.position.z = ps.transform.translation.z;
		newmsg.pose.pose.orientation = ps.transform.rotation;
		newmsg.header.stamp = trans.stamp_;

  	newmsg.header.frame_id = "/robot/base_link";
	  //robotodom_pub.publish(newmsg);
    tb->sendTransform( trans );

  }
  catch( tf::TransformException &ex )
  {
    ROS_WARN( "unable to do tranformation: [%s]", ex.what());
  }
}

void child_cb( const nav_msgs::OdometryConstPtr& msg )
{
  geometry_msgs::PoseStamped p;
  p.header.stamp = msg->header.stamp;
  p.pose = msg->pose.pose;
	//printf( "/child/odom: %f,%f,%f\n", p.pose.position.x, p.pose.position.y, p.pose.orientation.z );
     
  tf::Transform pt;
  tf::poseMsgToTF(p.pose,pt);
  tf::StampedTransform tx( pt,p.header.stamp,"/ovh", "/child/base_link");
  tb->sendTransform(tx);
}

void parent_cb( const nav_msgs::OdometryConstPtr & msg )
{

  geometry_msgs::PoseStamped p;
  p.header.stamp = msg->header.stamp;
  p.pose = msg->pose.pose;
 
	//printf( "/parent/odom: %f,%f,%f\n", p.pose.position.x, p.pose.position.y, p.pose.orientation.z );

  tf::Transform pt;
  tf::poseMsgToTF(p.pose,pt);
  tf::StampedTransform tx( pt,p.header.stamp,"/ovh", "/parent/base_link");
  tb->sendTransform(tx);   
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "stage_adapt" );
  ros::NodeHandle n;
  tb = new tf::TransformBroadcaster();
  tl = new tf::TransformListener();
  n.param( "robotname", robot_name_, std::string("/robot_0"));
  n.param( "childname", child_name_, std::string("/robot_1"));
  n.param( "parentname", parent_name_, std::string("/robot_2"));
  cmdvel_pub = n.advertise<geometry_msgs::Twist>(robot_name_+"/cmd_vel",1000);
  robotodom_pub = n.advertise<nav_msgs::Odometry>("/robot/odometry",1000);
  ros::Subscriber rpos_sub = n.subscribe( robot_name_ + "/odom", 1, odom_cb );
  ros::Subscriber rodom_sub = n.subscribe( robot_name_ + "/base_pose_ground_truth", 1, robot_cb );
  ros::Subscriber cpos_sub = n.subscribe( child_name_ + "/base_pose_ground_truth", 1, child_cb );
  ros::Subscriber ppos_sub = n.subscribe( parent_name_ + "/base_pose_ground_truth", 1,parent_cb );
  ros::Subscriber cmdvel_sub = n.subscribe("/robot/cmd_vel",1,cmdvel_cb);

  tf::Transform t(tf::Quaternion(0,0,0),tf::Point(0,0,0));

  ident = t;
  tf::Transform b(tf::Quaternion(0,0,0),tf::Point(0,0,0.5));
  bandit_offset = b;

  ros::Rate loop_rate(20);

  while( ros::ok() )
  {
/*
    stamp = ros::Time::now(); 
  	tf::Transform t(tf::Quaternion(0.0,0.0,0), tf::Point(0,0,0));
	  tf::StampedTransform tm( t, stamp, "/ovh","/map");
  	//tb->sendTransform(tm);
    tf::Quaternion quat;
    //quat.setRPY( 0, 0, 0 );
    quat.setRPY( M_PI, 0, 0 );
/ *
	  tf::Transform tt(quat, tf::Point(0,0,3.5));
  	tf::StampedTransform th( tt, stamp, "/ovh","/ovh_height");
	  tb->sendTransform(th);
* /
  	tf::Transform tb2(tf::Quaternion(0.0,0.0,0.0), tf::Point(0,0,0.5));
	  tf::StampedTransform thb( tb2, stamp, "/robot/base_link","bandit_torso_link");
	  //tb->sendTransform(thb);
*/
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  printf( "\nQuitting... \n" );
  return 0;
}
