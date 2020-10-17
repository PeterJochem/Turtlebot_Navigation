/** @file
 * @brief Launch robot in Gazebo and navigate to waypoints on the server  
 *
 * Parameters: /rotational_vel_limit: max rotational speed
 *             /trans_vel_limit: max translational speed
 *	       /k_p_trans: proportional gain for translating
 *	       /k_i_trans: integral gain for translating
 *	       /k_p_rot: proportional gain for rotating
 *	       /k_i_rot: integral gain for rotating
 *	       /linear_threshold: linear distance where we consider robot at a point
 *             /angular_threshold: angular distance where we consider robot'a angle to be equal
 *
 *
 * Publishes: /odom: standard ROS odometry message
 * 	      /visualization_marker: Marker for RVIZ to denote where the robot has been
 *
 * Subscribes: /joint_states: The current state of each of the robot's joints
 *
 * Services: /set_pose: Reset the robot to a new SE(2) configuration */

#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <tsim/PoseError.h>
#include <tsim/traj_reset.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "rigid2d/waypoints.hpp"
#include "rigid2d/setPose.h"
#include "nuturtle_robot/start_waypoints.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>

double max_rotation_speed, max_translational_speed, frac_rot_vel, frac_trans_vel;
double current_odom_x = 0.0; 
double current_odom_y = 0.0;
double current_odom_theta = 0.0;
double desired_angle = 0.0;
double linear_threshold, angular_threshold;
double angle_integral = 0.0;
double k_p_trans, k_i_trans, k_p_rot, k_i_rot;
bool hasStarted = false;
ros::ServiceClient set_pose_client;

/** @brief Implements a feedback controller for the Gazebo simulation */
class FSM_Feedback {

	bool translate;
	int timeQuantas;
	double frequency;	
	double starting_x;
	double starting_y;
	double pentagon_length;
	
	public:
		FSM_Feedback(double);
		FSM_Feedback();
		void createPentagonWayPoints();
		void setRates();
		std::tuple<double, double> nextWaypoint();
		geometry_msgs::Twist checkUpdate();
		void publishMarker();

		std::vector<double> waypoints; // (x1, y1), (x2, y2) ...
		int numWaypoints; // Number of (x, y) pairs in the above list
		int currentWaypoint;
		ros::NodeHandle n;
		int markerCount;
      		ros::Publisher marker_pub;
		std::string base_frame_id;
		rigid2d::WayPoints current_waypoints;		
		double rot_vel, trans_vel;	
		tf::TransformListener listener;
		void processMap(const nav_msgs::OccupancyGrid map);
		void updateWaypoints(const ros::TimerEvent&);
};

/** @brief Constructor for the FSM
 *  @param frequency - rate at which to update FSM. Optional. Defaults to 100.0 */
FSM_Feedback::FSM_Feedback(double frequency = 100.0) {

	translate = false;	
	timeQuantas = 0;
	this->frequency = frequency;
	createPentagonWayPoints();
	currentWaypoint = 1;
	
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	
	n.getParam("/base_frame_id", base_frame_id);
	markerCount = 0;

	auto [nextX, nextY] = nextWaypoint();
        desired_angle = current_odom_theta + std::atan2(nextY - current_odom_y, nextX - current_odom_x); 

}

/** @brief Default Constructor for the FSM */
FSM_Feedback::FSM_Feedback() {

        translate = false;
        timeQuantas = 0;
        this->frequency = 100.0;
        currentWaypoint = 1;

        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

        n.getParam("/base_frame_id", base_frame_id);
        markerCount = 0;

	auto [nextX, nextY] = nextWaypoint();
        desired_angle = current_odom_theta + std::atan2(nextY - current_odom_y, nextX - current_odom_x);
}

/** @brief Compute and return the next (x, y) pair to navigate to
 *  @return the next waypoint as a (x, y) tuple defined in the odom frame */
std::tuple<double, double> FSM_Feedback::nextWaypoint() {

	double nextX_map = waypoints.at(currentWaypoint * 2);
	double nextY_map = waypoints.at(currentWaypoint * 2 + 1);

	geometry_msgs::PointStamped map_goal;
	geometry_msgs::PointStamped odom_goal;
	map_goal.header.frame_id = "map";
	map_goal.header.stamp = ros::Time();

	map_goal.point.x = nextX_map;
	map_goal.point.y = nextY_map;
	map_goal.point.z = 0;

	// convert to the odom frame
	try{
     		listener.transformPoint("odom", map_goal, odom_goal);
    	}
    	catch (tf::TransformException &ex) {
      		ROS_ERROR("%s",ex.what());
    	}
	
	return {odom_goal.point.x, odom_goal.point.y};
}

/** @brief Set the robot's rotational and translational speeds */
void FSM_Feedback::setRates() {
	rot_vel = max_rotation_speed * frac_rot_vel; 
	trans_vel = max_translational_speed * frac_trans_vel; 	
}

/* @brief Publish a marker at the robot's location */
void FSM_Feedback::publishMarker() {

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


/** @brief Compute the 5 vertices of a pentagon with the given
 * 	   length between vertices. Write the list of points 
 * 	   to the parameter server */
void FSM_Feedback::createPentagonWayPoints() { 

	numWaypoints = 5;
	double angle = (2 * 3.14 / 5.0);
	double nextX, nextY;

	n.getParam("/pentagon_length_real_world", pentagon_length);
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

	// Removed for testing
	//n.setParam("/waypoints", waypoints); // Puts the points on the ROS server
}

/** @brief Ros timer callback. Updates the vector of waypoints 
 */
void FSM_Feedback::updateWaypoints(const ros::TimerEvent&) {
		
	n.getParam("/waypoints", waypoints); // vector of (x1, y1), (x2, y2), ...			
	numWaypoints = waypoints.size()/2;
}

/** @brief Check if we reached the waypoint and also
 * 	   compute our next twist 
 *  @return the next twist to be sent to the Turtlebot */
geometry_msgs::Twist FSM_Feedback::checkUpdate() {

	if (waypoints.size() == 0) { 
		std::cout << "No waypoints on the server" << std::endl;
		geometry_msgs::Twist zero_twist;
		return zero_twist;	
	}
	
	auto [nextX, nextY] = nextWaypoint();

	geometry_msgs::Twist nextTwist;
	nextTwist.linear.x = 0.0;
	nextTwist.linear.y = 0.0;
	nextTwist.linear.z = 0.0;

	nextTwist.angular.x = 0.0;
	nextTwist.angular.y = 0.0;
	nextTwist.angular.z = 0.0;
	
	// compute heading using the laser data rather than odometry
	double cartesian_error = sqrt((pow((nextX - current_odom_x), 2) + pow((nextY - current_odom_y), 2)));
        
	desired_angle = std::atan2(nextY - current_odom_y, nextX - current_odom_x);
	double angular_error = desired_angle - current_odom_theta;
		
	if (abs(angular_error) > angular_threshold) {
		translate = false;
        	timeQuantas = 0;
	}
	
	if (translate) {
		
		if (cartesian_error <= linear_threshold) {
			translate = false;
			timeQuantas = 0;
			publishMarker();
			currentWaypoint++;
			currentWaypoint = currentWaypoint % numWaypoints; 
					
			auto [nextX, nextY] = nextWaypoint();
			desired_angle = std::atan2(nextY - current_odom_y, nextX - current_odom_x);
		}
		else {
			nextTwist.linear.x = k_p_trans * cartesian_error;
			timeQuantas++;
		}
	}	
	else {

		if (abs(angular_error) <= angular_threshold) {
			translate = true;
			timeQuantas = 0;
			auto [nextX, nextY] = nextWaypoint();
			std::cout << "New waypoint is (" << nextX << ", " << nextY << ")" << std::endl;
			angular_error = 0;
		}
		else {
			
			// Fixes issue with discontinuity and finds shortest angular path			
			if (abs(angular_error) > 3.14) {
				// double larger = max(desired_angle, current_odom_theta)
				angular_error = (3.14 - desired_angle) + (current_odom_theta - 3.14); 	
			}

			nextTwist.angular.z = k_p_rot * angular_error; // + (angle_integral); // 1000.0; // k_p_rot * angular_error;	
			angle_integral = angle_integral + angular_error * k_i_rot;
			timeQuantas++;	
		}
	}
	
	return nextTwist;
}


/** @brief Read parameters from the server and print them to the log
 * 	   Helpful for debugging purposes */
void logParams(void) {

	double pentagon_length, rot_vel, trans_vel, starting_x, starting_y;
	ros::NodeHandle n;

	n.getParam("/pentagon_length_real_world", pentagon_length);
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

/** @brief Check that the user defined velocity is legal */
inline bool isVelocityIllegal(double velocity) {
        return (velocity < 0.0 || velocity > 1.0);
}

/** @brief Lets robot start navigating
 *  @param req has two fields req.clockwise and req.fraction_of_max_angular_velocity 
 * 	  clockwise indicates which direction to rotate in
 * 	  fraction of max velocity indicates the percent of the max angular velocity
 * 	  to rotate at. Defaults to one half max velocity if given value is not legal
 *  @param res is not used but is there for ROS to compile/be able to run
 *  @return true */
bool start_waypoints(nuturtle_robot::start_waypoints::Request &req, nuturtle_robot::start_waypoints::Response &res) {

        rigid2d::setPose newPose;
        newPose.request.x = 0.0;
        newPose.request.y = 0.0;
        newPose.request.theta = 0.0;
        set_pose_client.call(newPose);
	
	frac_rot_vel = req.frac_rot_vel;
	frac_trans_vel = req.frac_trans_vel;
        if (isVelocityIllegal(req.frac_rot_vel)) {
		frac_rot_vel = 0.5;
        }
	if (isVelocityIllegal(req.frac_trans_vel)) {
		frac_trans_vel = 0.5;
	}
        
        hasStarted = true;
        return true;
}


/** @brief Take the odometry data and update our local values 
 * 	   of where the robot is according to the odometry data
 * 	   Remeber that the odometry data is still just a prediction
 *         of where we are. It is not the ground truth 
 *  @param odom_data is the new odometry message */
void odomCallback(const nav_msgs::Odometry odom_data) {
	
	current_odom_x = odom_data.pose.pose.position.x;
	current_odom_y = odom_data.pose.pose.position.y;
	
	// Convert the quaternion to an angle 
	tf::Quaternion q(odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y,
        		 odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w);
    
	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;

	// yaw is rotation about the z-axis
    	m.getRPY(roll, pitch, yaw);
    	current_odom_theta = yaw;
}

// When we get new map data, we get a new transform from
// map to odom so we should relocalize ourselves
void FSM_Feedback::processMap(const nav_msgs::OccupancyGrid map) {
	
   	tf::StampedTransform transform;
	try {
		listener.lookupTransform("/odom", "/map", ros::Time(0), transform);
   	}
   	catch (tf::TransformException &ex) {
     		ROS_ERROR("%s",ex.what());
   	}
	
	// Convert quaternion to the robot's orientation in the plane	
	double roll, pitch, yaw; // yaw is rotation about the z-axis
   	tf::Quaternion myQ = transform.getRotation();
	tf::Matrix3x3 m(myQ);
    	m.getRPY(roll, pitch, yaw);

    	current_odom_theta = yaw;
    	current_odom_x = transform.getOrigin().x();
    	current_odom_y = transform.getOrigin().y();

    return;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "waypoint_node");
	ros::NodeHandle n;
	
	n.getParam("/rotational_vel_limit", max_rotation_speed);
        n.getParam("/trans_vel_limit", max_translational_speed);
	n.getParam("/k_p_trans", k_p_trans);
	n.getParam("/k_i_trans", k_i_trans);
	n.getParam("/k_p_rot", k_p_rot);
        n.getParam("/k_i_rot", k_i_rot);
	n.getParam("/linear_threshold", linear_threshold);
        n.getParam("/angular_threshold", angular_threshold);

	ros::service::waitForService("/set_pose", -1);
        set_pose_client = n.serviceClient<rigid2d::setPose>("/set_pose");	
        ros::ServiceServer start_service = n.advertiseService("start_waypoints", start_waypoints);

	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);

	int frequency; // This specifies the rate at which we loop - this means loop at 1000 Hz
	n.getParam("/frequency", frequency);
	ros::Rate loop_rate(frequency);

	FSM_Feedback myFSM = FSM_Feedback(double(frequency));
	ros::Subscriber map_sub = n.subscribe("/map", 1, &FSM_Feedback::processMap, &myFSM);

	turtlesim::Pose poseNow;
	int count = 0;

	while(!hasStarted) {
                ros::spinOnce();
        }
	ros::spinOnce();


	// testing, put the new waypoints onto the server. These are points in the map frame
	//std::vector<double> newPts = {0, 0, 0.5, 0.5, 0.5, 1, 1.0, 1.5};
	//n.setParam("/waypoints", newPts);	
	ros::Timer updateWayptsTimer = n.createTimer(ros::Duration(1.0), &FSM_Feedback::updateWaypoints, &myFSM);

	myFSM.setRates();
	while (ros::ok()) {
		geometry_msgs::Twist currentTwist = myFSM.checkUpdate(); 
		cmd_vel_pub.publish(currentTwist);
		ros::spinOnce();
		count++;
	}

	return 0;
}
