#include <ros/ros.h>
//#include <feature_extractor/CPRWFeatureVector.h>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>


int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "gmm_model");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::spin();
	return 0;
}
