#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
//#include "rigid2d/"

using namespace rigid2d;
ros::Subscriber cmd_vel_sub;
ros::Subscriber sensor_sub;
ros::Publisher wheel_pub;
ros::Publisher joints_pub;
DiffDrive robot;


/* Describe   
 */
void cmd_vel_callback(geometry_msgs::Twist twist) {
	// the twist is the desired twist
	// compute how to rotate the wheels in order to achieve it
	// clamp inputs etc
	// publish the wheel_command with wheel_pub	
	
	// WheelVelocities DiffDrive::twistToWheels(Twist2D tw)
		



}

/* Describe 
 */
void sensor_sub_callback(nuturtlebot::SensorData newData) {
	// newData is the encoder postion (and velocity????) of the wheels  	
	// compute and publish the angle (rads) and velocity on joints_pub  


}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "turtle_interface_node"); 
        ros::NodeHandle n;
	
	// Get values from the param server
	double wheel_base, wheel_radius, frequency;

        n.getParam("/wheel_base", wheel_base);
	n.getParam("/wheel_radius", wheel_radius);
	n.getParam("/frequency", frequency);
	
	// Log?	
	//ROS_INFO("/wheel_radius is %f", wheel_radius);
	//ROS_INFO("/wheel_base is %f", wheel_base);

	// Get the current pose? Does it even matter for this?
	robot = DiffDrive(Transform2D(), wheel_base, wheel_radius);
				


	cmd_vel_sub = n.subscribe("/turtle1/cmd_vel", 1, cmd_vel_callback);
	sensor_sub = n.subscribe("/sensor_sub", 1, sensor_sub_callback);
	
	// How big to make the queue?
	// /wheel_cmd or /turtle1/wheel_cmd?
	wheel_pub = n.advertise<geometry_msgs::Twist>("/wheel_cmd", 1);
	joints_pub = n.advertise<nuturtlebot::WheelCommands>("/joint_states", 1);
		

	// What to set this to?
  	ros::Rate loop_rate(10);

	return 0;
}
