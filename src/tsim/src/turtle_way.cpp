#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <tsim/PoseError.h>
#include <tsim/traj_reset.h>
#include <sstream>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include "rigid2d/waypoints.hpp"
//#include "rigid2d/rigid2d.hpp"

ros::ServiceClient teleport_client;
ros::ServiceClient setPen_client;
ros::ServiceClient reset_client;
turtlesim::Pose currentPose;

/* Describe this class
*/
class FSM_Feedforward {

	bool translate;
	int timeQuantas;
	double frequency;	
	double starting_x;
	double starting_y;
	double pentagon_length;
	double rot_vel;
	double trans_vel;

	public:
	FSM_Feedforward(double);
	void createPentagonWayPoints();
	std::tuple<double, double, double, double> nextWaypoint();
	geometry_msgs::Twist checkUpdate(turtlesim::Pose);
	std::vector<double> waypoints;
	int currentWaypoint;
	ros::NodeHandle n;
	
	int markerCount;
      	ros::Publisher marker_pub;
	void publishMarker();	
	std::string base_frame_id;

	// Use the waypoints.hpp file
	//  WayPoints(std::vector<Vector2D> points)
	rigid2d::WayPoints current_waypoints;		
};

/* This is the constructor for the FSM
*/
FSM_Feedforward::FSM_Feedforward(double frequency) {

	translate = false;	
	timeQuantas = 0;
	this->frequency = frequency;
	createPentagonWayPoints();
	currentWaypoint = 1;
	
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	
	n.getParam("/base_frame_id", base_frame_id);
	markerCount = 0;
}

/* Describe
*/
std::tuple<double, double, double, double> FSM_Feedforward::nextWaypoint() {

	double nextX = waypoints.at(currentWaypoint * 2);
	double nextY = waypoints.at(currentWaypoint * 2 + 1);
	double priorX, priorY;

	int maxIndex = waypoints.size() - 1;
	if (currentWaypoint == 0) {
		priorX = waypoints.at(maxIndex - 1);
		priorY = waypoints.at(maxIndex);
	}
	else {
		priorX = waypoints.at((currentWaypoint - 1) * 2);
		priorY = waypoints.at((currentWaypoint - 1) * 2 + 1);
	}

	return {nextX, nextY, priorX, priorY};
}

void FSM_Feedforward::publishMarker() {

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



/* Compute the 5 vertices of a pentagon with the given
 * length between vertices. Write the list of points 
 * to the parameter server
 */
void FSM_Feedforward::createPentagonWayPoints() { 

	double angle = (2 * 3.14 / 5.0);
	double nextX, nextY;

	n.getParam("/pentagon_length", pentagon_length);
	n.getParam("/x", starting_x);
	n.getParam("/y", starting_y);
	n.getParam("/rot_vel", rot_vel);
	n.getParam("/trans_vel", trans_vel);

	for (int i = 0; i < 5; i++) {
		nextX = starting_x + (pentagon_length * sin(angle * i));
		nextY = starting_y + (pentagon_length * cos(angle * i));  

		waypoints.push_back(nextX);
		waypoints.push_back(nextY);
	}

	n.setParam("/waypoints", waypoints);
}


/* Describe this method here
 * Inputs:
 * Returns: The next twist to be sent to the Turtlebot 
 */
geometry_msgs::Twist FSM_Feedforward::checkUpdate(turtlesim::Pose currentPose) {

	// Read the parameters from the server  
	std::string s;

	if (waypoints.size() == 0) {
		std::cout << "No waypoints on the server" << std::endl;
		// Throw error?
	}

	auto [nextX, nextY, priorX, priorY] = nextWaypoint();

	geometry_msgs::Twist nextTwist;
	nextTwist.linear.x = 0.0;
	nextTwist.linear.y = 0.0;
	nextTwist.linear.z = 0.0;

	nextTwist.angular.x = 0.0;
	nextTwist.angular.y = 0.0;
	nextTwist.angular.z = 0.0;

	if (translate) {

		// Distance = Rate * time
		//double schedLinear = pentagon_length / trans_vel; 
		double schedLinear = sqrt((pow((nextX - priorX), 2) + pow((nextY - priorY), 2))) / trans_vel;

		if ( (timeQuantas * (1.0/frequency) ) >= schedLinear ) {
			translate = false;
			timeQuantas = 0;
			publishMarker();
			currentWaypoint++;
			currentWaypoint = currentWaypoint % 5; 
		}
		else {
			timeQuantas++;
			nextTwist.linear.x = trans_vel;
		}
	}	
	else {
		// Distance = Rate * time
		double angle = (2 * 3.14 / 5.0); 
		double schedRotational = angle / rot_vel;

		if ( (timeQuantas * (1.0/frequency) ) >= schedRotational ) {
			translate = true;
			timeQuantas = 0;
		}
		else {
			nextTwist.angular.z = rot_vel;
			timeQuantas++;	
		}
	}

	return nextTwist;
}


/* Describe this method 
*/
void makeSetPen(bool on, turtlesim::SetPen& sp) {

	unsigned int a = 1;
	sp.request.r = a;
	sp.request.g = a;
	sp.request.b = a;
	sp.request.width = a;
	sp.request.off = !on;

	return;
}

/* This is the callback for the subscriber to the pose messages
 * of turtle1
 */
void poseCallback(const turtlesim::Pose::ConstPtr& pose) {

	currentPose = *pose;
	return;
}

/* Describe this function
*/
void logParams(void) {

	double pentagon_length;
	double rot_vel;
	double trans_vel;
	double starting_x;
	double starting_y;

	ros::NodeHandle n;

	n.getParam("/pentagon_length", pentagon_length);
	ROS_INFO("/pentagon_length is %f", pentagon_length);

	n.getParam("/rot_vel", rot_vel);
	ROS_INFO("/rot_vel is %f", rot_vel);

	n.getParam("/trans_vel", trans_vel);
	ROS_INFO("/trans_vel is %f", trans_vel);

	n.getParam("/x", starting_x);
	ROS_INFO("/x is %f", starting_x);

	n.getParam("/y", starting_y);
	ROS_INFO("/y is %f", starting_y);	
}

/* Add description
*/
int main(int argc, char **argv) {

	ros::init(argc, argv, "turtle_rect");

	ros::NodeHandle n;

	logParams();

	ros::service::waitForService("turtle1/teleport_absolute", -1); 
	teleport_client = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");              	

	// Lift the pen
	ros::service::waitForService("turtle1/set_pen", -1);
	setPen_client = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
	turtlesim::SetPen sp;
	makeSetPen(false, sp);
	setPen_client.call(sp);

	// Teleport the turtle to the start
	ros::service::waitForService("tsim/traj_reset", -1);
	ros::ServiceClient reset_client = n.serviceClient<tsim::traj_reset>("tsim/traj_reset");
	tsim::traj_reset srv;
	reset_client.call(srv);

	// Put pen down
	makeSetPen(true, sp);
	setPen_client.call(sp);

	// How big to make this queue 
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// Publishes the error between actual pose and the desired pose
	ros::Publisher error_pose_pub = n.advertise<tsim::PoseError>("turtle1/pose_error", 1000);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// Subscribe to the turtle's pose
	ros::Subscriber sub = n.subscribe("/turtle1/pose", 1, poseCallback);

	int frequency;
	n.getParam("/frequency", frequency);
	// This specifies the rate at which we loop - this means loop at 1000 Hz
	ros::Rate loop_rate(frequency);

	// Create our FSM to record state
	//FSM_Feedback myFSM = FSM_Feedback();
	FSM_Feedforward myFSM = FSM_Feedforward(double(frequency));

	turtlesim::Pose poseNow;
	int count = 0;

	while (ros::ok()) {

		// Get the turtle's current position
		// This is done by our subscription - it fills global variable
		// Copy the pose to a new variable to avoid asynchronous errors
		poseNow = currentPose;

		geometry_msgs::Twist currentTwist = myFSM.checkUpdate(poseNow); 

		// Compute the error between actual position and position it would 
		// be in if it had followed the command exactly
		tsim::PoseError current_error;
		current_error.x_error = ( (1 / float(frequency)) * currentTwist.linear.x) - poseNow.x;
		current_error.y_error = ( (1 / float(frequency)) * currentTwist.linear.y) - poseNow.y;
		current_error.theta_error = ( (1 / float(frequency)) * currentTwist.angular.z) - poseNow.theta; 

		// Publish the current error 
		error_pose_pub.publish(current_error);

		// Issue a new twist 
		cmd_vel_pub.publish(currentTwist);

		// Do I need this?
		// Yes, this is when callbacks get called
		ros::spinOnce();

		loop_rate.sleep();
		count++;
	}


	return 0;
}
