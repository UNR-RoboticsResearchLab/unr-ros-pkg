#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class StageBot
{
public:
	StageBot();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
	
};


StageBot::StageBot():
	linear_(1),
	angular_(2)
{
	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, 0.5);
	nh_.param("scale_linear", l_scale_, 0.6);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &StageBot::joyCallback, this);
}

void StageBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	if ( joy->buttons[4] == 1 && joy->buttons[5] == 1)
  {
    vel.angular.z = l_scale_*joy->axes[2];
    vel.linear.x = a_scale_*joy->axes[1];
    ROS_DEBUG( "xaxis: %f(%f), zrot: %f(%f)", joy->axes[1], vel.linear.x, joy->axes[2], vel.angular.z );
  }
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joystick");
	StageBot stage_bot;

	ros::spin();
}
