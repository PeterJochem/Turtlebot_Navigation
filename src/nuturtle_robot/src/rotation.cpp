#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuturtle_robot/start.h"

bool rotate = false;
bool hasStarted = false;
double rotation_speed = 0.0;

/* req is a single double which tells robot how fast to rotate
*/
bool start(nuturtle_robot::start::Request  &req, nuturtle_robot::start::Response &res) {

	rotation_speed = req.desired_angular_velocity;
	rotate = true;
	hasStarted = true;
	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "rotation");
	ros::NodeHandle n;
	
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::ServiceServer start_service = n.advertiseService("start", start);

	// Create a diff drive robot to record the position and number of rotations?
	
	// What frequency to publish at? Doesn't matter for this?
	ros::Rate loop_rate(10);

	while(!hasStarted) {
		ros::spinOnce();
	}

	while (rotate) {   

		geometry_msgs::Twist newTwist;
		newTwist.angular.z = rotation_speed;

		cmd_vel_pub.publish(newTwist); 

		// Count the number of rotations?
			
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
