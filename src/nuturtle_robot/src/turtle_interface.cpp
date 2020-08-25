#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdlib.h>
#include "rigid2d/waypoints.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"


/* Describe   
 */
void cmd_vel_callback(geometry_msgs::Twist twist) {

}

/* Describe 
 */
void sensor_sub_callback(nuturtlebot::SensorData newData) {

}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "turtle_interface_node"); 
        ros::NodeHandle n;
	
		
	ros::Subscriber cmd_vel_sub = n.subscribe("/turtle1/cmd_vel", 1, cmd_vel_callback);
	ros::Subscriber sensor_sub = n.subscribe("/sensor_sub", 1, sensor_sub_callback);
	
	// How big to make the queue?
	// /wheel_cmd or /turtle1/wheel_cmd?
	ros::Publisher wheel_pub = n.advertise<geometry_msgs::Twist>("/wheel_cmd", 1);
	ros::Publisher joints_pub = n.advertise<nuturtlebot::WheelCommands>("/joint_states", 1);
		

  	ros::Rate loop_rate(10);

	return 0;
}
