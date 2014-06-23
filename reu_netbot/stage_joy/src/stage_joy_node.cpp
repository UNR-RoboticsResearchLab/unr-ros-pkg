#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class StageBot
{
public:
	StageBot();
	void spin();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

  double pub_rate_;
  bool deadman_;
	geometry_msgs::Twist vel_;
};

StageBot::StageBot():
	linear_(1),
	angular_(3),
  l_scale_(0.6),
  a_scale_(0.3),
  deadman_(false)
{
	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("pub_rate", pub_rate_, 10.0);
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &StageBot::joyCallback, this);

  vel_.linear.x = 0.0;
  vel_.angular.z = 0.0;
}

void StageBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // joy_node only sends messages when changed, so need to store vels in class variables in order to publish intermittently

	if ( joy->buttons[4] == 1 && joy->buttons[5] == 1)
  {
    deadman_ = true;
  }
  else
  {
    deadman_ = false;
  }
  vel_.angular.z = l_scale_*joy->axes[angular_];
  vel_.linear.x = a_scale_*joy->axes[linear_];
  ROS_DEBUG( "xaxis: %f(%f), zrot: %f(%f)", joy->axes[linear_], vel_.linear.x, joy->axes[angular_], vel_.angular.z );

}

void StageBot::spin()
{
  ros::Rate loop_rate(pub_rate_);

  while( ros::ok() )
  {
    if( deadman_ )
    {
      vel_pub_.publish(vel_);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "joystick");
	StageBot stage_bot;
  stage_bot.spin();
}
