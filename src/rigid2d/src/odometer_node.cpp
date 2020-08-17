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
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "rigid2d/diff_drive.hpp"

namespace rigid2d {
}

std::string odom_frame_id, body_frame_id, left_wheel_joint, 
	right_wheel_joint, base_frame_id;

double wheel_base, wheel_radius, frequency;

ros::NodeHandle n;
tf2_ros::TransformBroadcaster odom_broadcaster;
ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

// Use to simulate robot internally
rigid2d::DiffDrive robot;

/* Describe
 * Check for no match case?
 */
std::tuple<double, double> getWheelPositions(sensor_msgs::JointState& current_joint_state) {
		
	double left_wheel_rads, right_wheel_rads;	
	
	for (unsigned int i = 0; i < current_joint_state.position.size(); i++) {
		if (left_wheel_joint == current_joint_state.name[i]) {
                        left_wheel_rads = current_joint_state.position[i];
                        break;
                }
	}
	
	for (unsigned i = 0; i < current_joint_state.position.size(); i++) {	
		if (right_wheel_joint == current_joint_state.name[i]) {
			right_wheel_rads = current_joint_state.position[i];
			break;	
		}
	}
			
	return {left_wheel_rads, right_wheel_rads};
}


void callback(sensor_msgs::JointState current_joint_state) {

	// Find the joint state position for the left wheel and right wheel
	auto [left_wheel_rads, right_wheel_rads] = getWheelPositions(current_joint_state); 

		
	// Update the odometer of the robot using the new wheel positions
	rigid2d::WheelVelocities cmd = robot.updateOdometry(left_wheel_rads, right_wheel_rads);
      	rigid2d::Twist2D twist = robot.wheelsToTwist(cmd);
	
	geometry_msgs::Pose2D pose = robot.pose();
      	tf2::Quaternion q;
      	q.setRPY(0, 0, pose.theta);
	
	nav_msgs::Odometry odom;
      	geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
      	odom.header.stamp = ros::Time::now();
      	odom.header.frame_id = odom_frame_id;	
	
	odom.pose.pose.position.x = pose.x;
      	odom.pose.pose.position.y = pose.y;
      	odom.pose.pose.position.z = 0.0;
      	odom.pose.pose.orientation = q_msg;
		
      	odom.child_frame_id = base_frame_id;
      	odom.twist.twist.linear.x = twist.dx;
      	odom.twist.twist.linear.y = twist.dy;
      	odom.twist.twist.angular.z = twist.w;

      	odom_pub.publish(odom);
	
	// Publish the transform
	geometry_msgs::TransformStamped T;
	T.header.stamp = ros::Time::now();
      	T.header.frame_id = odom_frame_id;

      	T.child_frame_id = base_frame_id;

      	T.transform.translation.x = pose.x;
      	T.transform.translation.y = pose.y;
      	T.transform.translation.z = 0.0;

      	T.transform.rotation.x = q.x();
      	T.transform.rotation.y = q.y();
      	T.transform.rotation.z = q.z();
      	T.transform.rotation.w = q.w();

      	odom_broadcaster.sendTransform(T);	
}

int main(int argc, char** argv) {
	
	// Init ROS Node
	ros::init(argc, argv, "odometry_node");
	
	// Pull data from the ROS server
	while ((!n.hasParam("wheel_base")) && (!n.hasParam("wheel_radius"))) {
  		// Wait for the parameter to be on the server
		;	
	}
	
	// Check that they are on the server?
	n.getParam("/wheel_base", wheel_base);
	n.getParam("/wheel_radius", wheel_radius);			
	n.getParam("/odom_frame_id", odom_frame_id);
 	n.getParam("/base_frame_id", base_frame_id);
    	n.getParam("/left_wheel_joint", left_wheel_joint);
    	n.getParam("/right_wheel_joint", right_wheel_joint);
    	n.getParam("/frequency", frequency);
	

	// Create a DiffDrive robot to simulate the robot
        // DiffDrive::DiffDrive(Transform2D pose, double wheel_base, double wheel_radius)
        robot = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base, wheel_radius);
	
	ros::Rate r(frequency);

	// Setup the subscriber
	ros::Subscriber sub = n.subscribe("joint_states", 1, callback);		

	ros::spin();
}
