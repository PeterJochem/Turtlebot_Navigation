/// \brief describe this file 

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
//#include "rigid2d/diff_drive.hpp"

namespace rigid2d {
}

std::string odom_frame_id = "aName"; // The name of the odometry tf frame
std::string body_frame_id; // The name of the body tf frame
std::string left_wheel_joint; // The name of the left wheel joint
std::string right_wheel_joint; // The name of the right wheel joint

// Use to simulate robot internally
rigid2d::DiffDrive myDiffDrive;

void callback(sensor_msgs::JointState current_joint_state) {

	// Update internal odometry node (use diff drive?)
	
	// Publish a nav_msgs/Odometry message
	
	// Broadcast the transform between odom_frame_id and the body_frame_id on /tf
		

}

int main(int argc, char** argv) {
	
	// Init ROS Node
	ros::init(argc, argv, "odometry_node");
	
	// Pull data from the ROS server
	// wheel base, wheel radius ...

 	ros::NodeHandle n;
  	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
 	tf2_ros::TransformBroadcaster odom_broadcaster;
		
	// Setup subscribers 
	ros::Subscriber sub = n.subscribe("joint_states", 1000, callback);		

	// Create a DiffDrive robot to simulate the robot
	// DiffDrive::DiffDrive(Transform2D pose, double wheel_base, double wheel_radius)
	myDiffDrive = rigid2d::DiffDrive();
			

	

}
