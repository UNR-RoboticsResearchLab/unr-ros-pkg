#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

	// Constants
        const float limits[17] = {       0.7,        // Diff of limits, wristThumbKnuck1
                                         0.3,        // Diff of limits, wristPinkyKnuck1
                                         0.23,       // Diff of limits, wristRingKnuck1
                                        -1.74533,   // Diff of limits, indexKnuck1
                                        -1.4        // Diff of limits, indexKnuck2
                                        -0.87,      // Diff of limits, indexKnuck3
                                        -1.74533,   // Diff of limits, middleKnuck1
                                        -1.4,       // Diff of limits, middleKnuck2
                                        -0.87,      // Diff of limits, middleKnuck3
                                        -1.74533,   // Diff of limits, ringKnuck1
                                        -1.4,       // Diff of limits, ringKnuck2
                                        -0.87,      // Diff of limits, ringKnuck3
                                        -1.74533,   // Diff of limits, pinkyKnuck1
                                        -1.4,       // Diff of limits, pinkyKnuck2
                                        -0.87,      // Diff of limits, pinkyKnuck3
                                        -1.4,       // Diff of limits, thumbKnuck1
                                        -1.13       // Diff of limits, thumbKnuck2
                                };

	// Robot state
	float wristThumbKnuck1 = 0, wristPinkyKnuck1 = 0, wristRingKnuck1 = 0,\ 
            indexKnuck1 = 0, indexKnuck2 = 0, indexKnuck3 = 0, middleKnuck1 = 0,\
            middleKnuck2 = 0, middleKnuck3 = 0, ringKnuck1 = 0, ringKnuck2 = 0,\
	    ringKnuck3 = 0, pinkyKnuck1 = 0, pinkyKnuck2 = 0, pinkyKnuck3 = 0,\
	    thumbKnuck1 = 0, thumbKnuck2 = 0, angle = 0;
	
	// Other variables
	float rat = 0; //to define the rate of closing
	int sign = -1; //to know if we are opening or closing

	// Message Declarations
    sensor_msgs::JointState joint_state;

	// loop through basic motion
	while (ros::ok()){
	// Update joint_state
		joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(17);
        joint_state.position.resize(17);        
        joint_state.name[0] ="wristThumbKnuck1";
        joint_state.position[0] = wristThumbKnuck1;
        joint_state.name[1] ="wristPinkyKnuck1";
        joint_state.position[1] = wristPinkyKnuck1;
        joint_state.name[2] ="wristRingKnuck1";
        joint_state.position[2] = wristRingKnuck1;
        joint_state.name[3] ="indexKnuck1";
        joint_state.position[3] = indexKnuck1;
        joint_state.name[4] ="indexKnuck2";
        joint_state.position[4] = indexKnuck2;
        joint_state.name[5] ="indexKnuck3";
        joint_state.position[5] = indexKnuck3;
        joint_state.name[6] ="middleKnuck1";
        joint_state.position[6] = middleKnuck1;
        joint_state.name[7] ="middleKnuck2";
        joint_state.position[7] = middleKnuck2;
        joint_state.name[8] ="middleKnuck3";
        joint_state.position[8] = middleKnuck3;
        joint_state.name[9] ="ringKnuck1";
        joint_state.position[9] = ringKnuck1;
        joint_state.name[10] ="ringKnuck2";
        joint_state.position[10] = ringKnuck2;
        joint_state.name[11] ="ringKnuck3";
        joint_state.position[11] = ringKnuck3;
        joint_state.name[12] ="pinkyKnuck1";
        joint_state.position[12] = pinkyKnuck1;
        joint_state.name[13] ="pinkyKnuck2";
        joint_state.position[13] = pinkyKnuck2;
        joint_state.name[14] ="pinkyKnuck3";
        joint_state.position[14] = pinkyKnuck3;
        joint_state.name[15] ="thumbKnuck1";
        joint_state.position[15] = thumbKnuck1;
        joint_state.name[16] ="thumbKnuck2";
        joint_state.position[16] = thumbKnuck2;
		
		// Update transform
		joint_pub.publish(joint_state);
		
		// Check limit (when hand is opened we should close and vise versa)
		 if(wristThumbKnuck1 >= limits[0] || wristThumbKnuck1 <= 0)
	 {
	       sign = -1 * sign;
	   }
	   rat = rat + (sign*0.01) ; //we devide the position to 100 positions (each part according to its limit)
	   
		wristThumbKnuck1 = rat * 0.7;  //should be rat + limits[0] ... but it had some errors on the hand
		wristPinkyKnuck1 = rat * 0.3; 
		wristRingKnuck1 = rat * 0.23;
        indexKnuck1 = rat * -1.74533;
        indexKnuck2 = rat * -1.4; 
        indexKnuck3 = rat * -0.87; 
        middleKnuck1 = rat * -1.74533;
        middleKnuck2 = rat * -1.4; 
        middleKnuck3 = rat * -0.87; 
        ringKnuck1 = rat * -1.74533;
        ringKnuck2 = rat * -1.4;
	    ringKnuck3 = rat * -0.87; 
	    pinkyKnuck1 = rat * -1.74533; 
	    pinkyKnuck2 = rat * -1.4; 
	    pinkyKnuck3 = rat * -0.87;
	    thumbKnuck1 = rat * -1.4; 
	    thumbKnuck2 = rat * -1.13;

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
	
	return 0;
}
