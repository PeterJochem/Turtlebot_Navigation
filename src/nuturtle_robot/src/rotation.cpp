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


int main(int argc, char **argv) {
  
  ros::init(argc, argv, "rotation");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
     
     geometry_msgs::Twist newTwist;
     newTwist.angular.z = 1.0;
   
     cmd_vel_pub.publish(newTwist); 

     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
