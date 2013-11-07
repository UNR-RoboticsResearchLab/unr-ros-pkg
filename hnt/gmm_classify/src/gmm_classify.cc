#include <ros/ros.h>

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "gmm_model");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::spin();
	return 0;
}
