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
#include <visualization_msgs/Marker.h>
#include "rigid2d/setPose.h"

namespace rigid2d {
}

class odometer {

	public:
		ros::NodeHandle n;			
		odometer(); // Default constructor
		std::string odom_frame_id, body_frame_id, left_wheel_joint, 
			right_wheel_joint, base_frame_id;

		double wheel_base, wheel_radius, frequency;
		int markerCount;
		ros::Publisher marker_pub;

		tf2_ros::TransformBroadcaster odom_broadcaster;
		ros::Publisher odom_pub;
		ros::Subscriber sub;

		// Use to simulate robot internally
		rigid2d::DiffDrive robot;
		ros::Timer timer;
		ros::ServiceServer set_pose;		
		bool set_pose_srv(rigid2d::setPose::Request &req, rigid2d::setPose::Response &res);
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

		/* Describe 
		*/
		void publishMarker(const ros::TimerEvent&) {

			// Set our initial shape type to be a cube
			uint32_t shape = visualization_msgs::Marker::CUBE;
			visualization_msgs::Marker marker;
			// Set the frame ID and timestamp.  See the TF tutorials for information on these.
			marker.header.frame_id = base_frame_id;
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.  This serves to create a unique ID
			// Any marker sent with the same namespace and id will overwrite the old one
			marker.ns = "basic_shapes";
			marker.id = markerCount;
			markerCount++;
			// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			marker.type = shape;

			// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
			marker.action = visualization_msgs::Marker::ADD;

			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			marker.pose.position.x = 0.0;
			marker.pose.position.y = 0.0;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;

			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 1.0f;
			marker.color.a = 0.5;

			marker.lifetime = ros::Duration();	
			marker_pub.publish(marker);
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
};


// Default constructor
odometer::odometer() {

	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	markerCount = 0;

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

	robot = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base, wheel_radius);

	ros::Rate r(frequency);

	// Setup the subscriber
	sub = n.subscribe("joint_states", 1, &odometer::callback, this);
	
	set_pose = n.advertiseService("set_pose", &odometer::set_pose_srv, this); 	
	
	//timer = n.createTimer(ros::Duration(3.0), &odometer::publishMarker, this);
}

/* Describe 
 */
bool odometer::set_pose_srv(rigid2d::setPose::Request &req, rigid2d::setPose::Response &res) {
	
	robot.setPose(req.x, req.y, req.theta);
	return true;
}

int main(int argc, char** argv) {

	// Init ROS Node
	ros::init(argc, argv, "odometer_node");
		
	ros::NodeHandle n;
	
	odometer myOdom = odometer();	
	
	ros::spin();
}
