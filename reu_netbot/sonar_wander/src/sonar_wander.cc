#include <ros/ros.h>
#include <p2os_msgs/SonarArray.h>
#include <geometry_msgs/Twist.h>

// these global variables are what you will set to complete the project
double front_dist;  // if set correctly, this will be the closest obstacle to the front of the robot
double left_dist; // if set correctly, this will be the closest obstacle to the left of the robot
double right_dist; // if set correctly, this will be the closest obstacle to the right of the robot

double getX(int i)
{
	if( i == 0 || i == 7 ) return .115;
	if( i == 1 || i == 6 ) return .155;
	if( i == 2 || i == 5 ) return .190;
	if( i == 3 || i == 4 ) return .210;

	return 0;
}

double getY(int i)
{
	double ret = 0;
	if( i == 0 || i == 7 ) return .130;
	if( i == 1 || i == 6 ) return .115;
	if( i == 2 || i == 5 ) return .080;
	if( i == 3 || i == 4 ) return .025;

	if( i > 3 ) ret = -ret;
	return ret;
}

double getT(int i)
{
	double ret = 0;
	if( i == 0 || i == 7 ) return 90;
	if( i == 1 || i == 6 ) return 50;
	if( i == 2 || i == 5 ) return 30;
	if( i == 3 || i == 4 ) return 10;

	if( i > 3 ) ret = -ret;
	return ret;

}

void sonarCallback( const p2os_msgs::SonarArray::ConstPtr &msg)
{
	front_dist = DBL_MAX;
	left_dist = DBL_MAX;
	right_dist = DBL_MAX;

	// get flbr values from sonar msgs
	if( msg->ranges_count != 16 && msg->ranges_count != 8 )
	{
		// number of sonar is not right, report things
		ROS_WARN("range_count is [%d] should be 8 or 16", msg->ranges_count );
	}
	
	// for each sonar reading
	for( int i = 0; i < 8; i++ )
	{
		// get xyt coord for sonar
		double x = getY(i);
		double y = getX(i);
		double t = M_PI * getT(i) / 180.0;

		// trace range from sonar
		x += msg->ranges[i] * sin( t );
		y += msg->ranges[i] * cos( t );	

		// update flbr reading
		if( x > -0.3 && x < 0.3 && y < front_dist )
		{
      // if so, set the front_distance to be
      // the current distance
			front_dist = y;
		}
		
    // is the x/y coordinate for this range point
    // closer than the closest right point so far?
    if( y > 0.25 && x > 0 )
    {
      // if so, set the right_distance to be
      // the current distance
      double dist = sqrt (y*y + x*x);
      if( dist < right_dist )
        right_dist = dist;
    }

    // is the x/y coordinate for this range point
    // closer than the closest left point so far?
    if( y > 0.25 && x < 0 )
    {
      // if so, set the left_distance to be
      // the current distance
      double dist = sqrt (y*y + x*x);
      if( dist < left_dist )
        left_dist = dist;
		}

	}
}


int main( int argc, char* argv[] )
{
	// listen to sonar_msg and output cmd_vel
  ros::init(argc,argv,"sonar_wander");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber sonar_sub = nh.subscribe("/sonar", 1000, sonarCallback);
  ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  geometry_msgs::Twist cmd_vel_msg;

  double des_vel = 1.0;

	// while 1
	while( ros::ok() )
	{
		double lvel = des_vel;
		double rvel = 0;
    printf( "front_dist: %0.2f left_dist: %0.2f right_dist: %0.2f\n", front_dist, left_dist, right_dist );

		//for flbr values
		// if too short in front
    if( front_dist < 1.25 )
      lvel = front_dist - 0.85;
			// slow down
		// if l > r + e, turn left

      if( left_dist < right_dist + 0.1 && left_dist < 1.0 )
        rvel = 0.75;

		// else if r > l + e turn right

      if( right_dist < left_dist + 0.1 && right_dist < 1.0 )
        rvel = -0.75;

		// else go straight
    // send the speeds to the robot
    cmd_vel_msg.linear.x = lvel;
    cmd_vel_msg.angular.z = rvel;
    cmd_vel.publish(cmd_vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
	}
	return 0;
}
