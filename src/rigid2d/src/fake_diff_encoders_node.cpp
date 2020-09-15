/// \brief fake_diff_encoders_node.cpp is a ROS node 
// that listens for twists for the robot to follow and 
// simulates what the encoder data should look like if we 
// followed the twist for a unit of time.  
//
// Subscibes: 
//
// Publishes:  

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "rigid2d/diff_drive.hpp"


rigid2d::Twist2D desired_twist;
std::string left_wheel_joint, right_wheel_joint;
double wheel_base, wheel_radius, frequency;
bool dataPresent = false;

/* Once a cmd_vel twist is received, the robot follows that 
 * velocity until the next cmd_vel command. Publish on joint_states 
 * assuming the robot followed cmd_vel for a whole unit of time
 */
void callback(geometry_msgs::Twist twist) {
	
	dataPresent = true;

	// Convert from a geometry_msgs twist to a rigid2d::Twist2D	
	desired_twist = rigid2d::convert3DTo2D(twist);
}

int main(int argc, char** argv) {
	
	// Init ROS Node
        ros::init(argc, argv, "fake_diff_encoders_node");		
	
	ros::NodeHandle n;
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	rigid2d::DiffDrive robot;

	// Read left_wheel_joint and right_wheel_joint
	// Check that they are on the server?
        n.getParam("/wheel_base", wheel_base);
        n.getParam("/wheel_radius", wheel_radius);
        n.getParam("/left_wheel_joint", left_wheel_joint);
	n.getParam("/right_wheel_joint", right_wheel_joint);
	n.getParam("/frequency", frequency);				
		
	robot = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base, wheel_radius);
	
	ros::Rate r(frequency);

	// Subscribe to geometry_msgs/Twist messages on cmd_vel
	// Queue size????? FIX ME!!!!!
	ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 1, callback);	
	
	while (ros::ok()) {		
		if (dataPresent) {	
			sensor_msgs::JointState current_joint_state;

	        	current_joint_state.header.stamp = ros::Time::now();
        		// current_joint_state.header.frame_id = ??;
        
        		current_joint_state.name.push_back(left_wheel_joint);
        		current_joint_state.name.push_back(right_wheel_joint);

			double dt = (double)(1.0/frequency);
			
			// Scale the twist!!
			rigid2d::Twist2D scaledTwist = desired_twist.scaleTwist(dt);
			robot.feedforward(scaledTwist);
				
			auto [encoder_left, encoder_right] = robot.getEncoders(); 
        		current_joint_state.position.push_back(encoder_left);
        		current_joint_state.position.push_back(encoder_right);               

	        	// Leave blank? velocity array, effort array
        		joint_state_pub.publish(current_joint_state);
			
			dataPresent = false;
		}

		ros::spinOnce();
      		r.sleep();	
	}

	ros::spin();
	return 0;
}
