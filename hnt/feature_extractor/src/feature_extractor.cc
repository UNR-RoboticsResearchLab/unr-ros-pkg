#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <point_map/PointMap.h>
#include <feature_extractor/CPRWFeatureVector.h>

bool got_map;
point_map::PointMap::Response map;

void
get_map()
{
  point_map::PointMap::Request  req;
  if( ros::service::call("/map_server", req,map) )
  {
    got_map = true;
		ROS_INFO( "got map" );
  }
}

double wall_state( double cx, double cy )
{
	double ret = 999;
	for( int i = 0; i < map.num_points; i++ )
	{
    // for each wall
    int i2 = (i+1) % map.num_points;

    // x1,y1 = wall pos[i];
    double x1 = map.x[i];
    double y1 = map.y[i];

    // x2,y2 = wall pos[i2];
    double x2 = map.x[i2];
    double y2 = map.y[i2];

    // x3,y3 = child pos;
    double x3 = cx;
    double y3 = cy;
    double u = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1))/(hypot(y2-y1,x2-x1)*hypot(y2-y1,x2-x1));
    if( u > 1.0 ) u = 1.0;
    if( u < 0.0 ) u = 0.0;

    // x4,y4 is the point on the wall closest to p3
    double x4 = x1 + u*(x2-x1);
    double y4 = y1 + u*(y2-y1);

    double r = hypot(y4-y3,x4-x3);
    if( r < ret ) ret = r;
	}

	return ret;
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "feature_extractor" );
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	tf::TransformListener tl;

	got_map = false;

	ros::Publisher feature_pub = nh.advertise<feature_extractor::CPRWFeatureVector>("features",10);

	feature_extractor::CPRWFeatureVector feature;
	feature.labels.push_back("dr");
	feature.labels.push_back("dw");
	feature.labels.push_back("dp");
	feature.labels.push_back("drb");
	feature.labels.push_back("v");
	feature.labels.push_back("vr");
	feature.labels.push_back("vp");
	feature.labels.push_back("vrb");

	while( ros::ok() )
	{
		loop_rate.sleep();
		ros::spinOnce();

		ros::Time past;
		ros::Time current;
		if( !got_map ) get_map();
		else 
		{
			tf::StampedTransform ctrans, rtrans, ptrans;
			tf::StampedTransform octrans, ortrans, optrans;
			try {
				tl.lookupTransform( "/ovh","/child/base_link", ros::Time(0), ctrans );
				tl.lookupTransform( "/ovh","/robot/base_link", ros::Time(0), rtrans );
				tl.lookupTransform( "/ovh","/parent/base_link", ros::Time(0), ptrans );
	
				past = ros::Time::now() - ros::Duration(0.25);

				tl.lookupTransform( "/ovh","/child/base_link", past, octrans );
				tl.lookupTransform( "/ovh","/robot/base_link", past, ortrans );
				tl.lookupTransform( "/ovh","/parent/base_link", past, optrans );
			
			}
			catch (tf::TransformException &ex ) {
				ROS_WARN( "unable to do transformation: [%s]", ex.what() );
				continue;
			}

			// get values from transforms
			double cx = ctrans.getOrigin().x();
			double cy = ctrans.getOrigin().y();
			double rx = rtrans.getOrigin().x();
			double ry = rtrans.getOrigin().y();
			double rt, roll, pitch;
			rtrans.getBasis().getEulerYPR( rt, pitch, roll );
			double px = ptrans.getOrigin().x();
			double py = ptrans.getOrigin().y();

			double ocx = octrans.getOrigin().x();
			double ocy = octrans.getOrigin().y();
			double orx = ortrans.getOrigin().x();
			double ory = ortrans.getOrigin().y();
			double ort;
			ortrans.getBasis().getEulerYPR( ort, pitch, roll );
			double opx = optrans.getOrigin().x();
			double opy = optrans.getOrigin().y();

			double dt = (ctrans.stamp_ - past).toSec();
			if( !dt > 0 )
			{
				ROS_WARN( "timestamps did not come out right" );
				continue;
			}

			// get immediate distance features
			double dr = hypot( cy-ry, cx-rx );
			double dw = wall_state(cx,cy);
			double dp = hypot( cy-py, cx-px );
			double drb = atan2( cy-ry, cx-rx ) - rt;
			while( drb >  M_PI ) drb-= 2*M_PI;
			while( drb < -M_PI ) drb+= 2*M_PI;

			// now get velocity features
			double v = hypot( cx-ocx, cy-ocy ) / dt;
			double vr = (hypot(cx-orx,cy-ory)-hypot(ocx-orx,ocy-ory)) / dt;
			double vp = (hypot(cx-opx,cy-opy)-hypot(ocx-opx,ocy-opy)) / dt;
			double vrb = fabs(atan2(ocy-ory,ocx-orx) - ort);
			while( vrb >  M_PI ) vrb-= 2*M_PI;
			while( vrb < -M_PI ) vrb+= 2*M_PI;
			vrb = fabs(vrb) - fabs(drb);
			vrb = vrb / dt;

			feature.values.clear();
			feature.values.push_back(dr);
			feature.values.push_back(dw);
			feature.values.push_back(dp);
			feature.values.push_back(drb);
			feature.values.push_back(v);
			feature.values.push_back(vr);
			feature.values.push_back(vp);
			feature.values.push_back(vrb);

			feature.header.stamp = ctrans.stamp_;

			feature_pub.publish(feature);

			ROS_INFO( "robot: %0.2f wall: %0.2f parent: %0.2f bearing: %0.2f", dr, dw, dp, drb*180.0/M_PI );
			ROS_INFO( "velocity: %0.2f vt: %0.2f vp: %0.2f vrb: %0.2f dt: %0.2f", v, vr, vp, vrb*180.0/M_PI, dt );

			// push feature set
		}

	}

	return 0;
}
