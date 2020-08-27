#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<std_:x
	  msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
