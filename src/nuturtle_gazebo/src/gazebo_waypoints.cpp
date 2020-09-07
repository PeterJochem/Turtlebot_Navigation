#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <tsim/PoseError.h>
#include <tsim/traj_reset.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include "rigid2d/waypoints.hpp"
#include "rigid2d/setPose.h"
#include "nuturtle_robot/start_waypoints.h"
//#include "rigid2d/rigid2d.hpp"

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

bool wp1 = false;

/* Describe this class
*/
class FSM_Feedforward {

	bool translate;
	int timeQuantas;
	double frequency;	
	double starting_x;
	double starting_y;
	double pentagon_length;
	
	public:
		double rot_vel;
        	double trans_vel;

		FSM_Feedforward(double);
		FSM_Feedforward();
		void createPentagonWayPoints();
		void setRates();
		std::tuple<double, double> nextWaypoint();
		geometry_msgs::Twist checkUpdate();
		std::vector<double> waypoints;
		int currentWaypoint;
		ros::NodeHandle n;
	
		int markerCount;
      		ros::Publisher marker_pub;
		void publishMarker();	
		std::string base_frame_id;
		rigid2d::WayPoints current_waypoints;		
};

/* This is the constructor for the FSM
*/
FSM_Feedforward::FSM_Feedforward(double frequency = 100.0) {

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
/* This is the constructor for the FSM
*/
FSM_Feedforward::FSM_Feedforward() {

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

/* Describe
*/
std::tuple<double, double> FSM_Feedforward::nextWaypoint() {
	

	// FIX ME FIX ME FIX ME FIX ME - shpuld loop back to the list start
	double nextX = waypoints.at(currentWaypoint * 2);
	double nextY = waypoints.at(currentWaypoint * 2 + 1);

	return {nextX, nextY};
}

void FSM_Feedforward::setRates() {
	rot_vel = max_rotation_speed * frac_rot_vel; 
	trans_vel = max_translational_speed * frac_trans_vel; 	
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

	n.setParam("/waypoints", waypoints);
}

/* Describe this method here
 * Inputs:
 * Returns: The next twist to be sent to the Turtlebot 
 */
geometry_msgs::Twist FSM_Feedforward::checkUpdate() {

	// Read the parameters from the server  
	std::string s;

	if (waypoints.size() == 0) {
		std::cout << "No waypoints on the server" << std::endl;
		// Throw error?
	}

	auto [nextX, nextY] = nextWaypoint();

	// double desired_angle = current_odom_theta + std::atan2(nextY - current_odom_y, nextX - current_odom_x); 
	// double desired_angle = 0.52 * currentWaypoint;

	geometry_msgs::Twist nextTwist;
	nextTwist.linear.x = 0.0;
	nextTwist.linear.y = 0.0;
	nextTwist.linear.z = 0.0;

	nextTwist.angular.x = 0.0;
	nextTwist.angular.y = 0.0;
	nextTwist.angular.z = 0.0;
		
	double cartesian_error = sqrt((pow((nextX - current_odom_x), 2) + pow((nextY - current_odom_y), 2)));
        double angular_error = desired_angle - current_odom_theta;
	
		
	//std::cout << "The next point is (" << nextX << ", " << nextY << ")" << std::endl;
	//std::cout << "The desired angle is " << rigid2d::rad2deg(desired_angle) << " degrees" << std::endl;
	//std::cout << "The current odom angle is " << rigid2d::rad2deg(current_odom_theta) << " degrees" << std::endl;
	std::cout << "The cartesian error is " << cartesian_error << std::endl;
	std::cout << "The angular error is " << angular_error << std::endl;	

	if (translate) {
			
		if (cartesian_error <= linear_threshold) {
			translate = false;
			timeQuantas = 0;
			publishMarker();
			currentWaypoint++;
			currentWaypoint = currentWaypoint % 5; 
			
			auto [nextX, nextY] = nextWaypoint();
			desired_angle = std::atan2(nextY - current_odom_y, nextX - current_odom_x);
			
			//wp1 = true;
			std::cout << "The desired angle is " << desired_angle << " degrees" << std::endl;
		}
		else {
			//if (!wp1) {
				nextTwist.linear.x = k_p_trans * cartesian_error;
			//}
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
			
			nextTwist.angular.z = k_p_rot * angular_error; //+ (angle_integral); // 1000.0; // k_p_rot * angular_error;
			angle_integral = angle_integral + angular_error * k_i_rot;
			timeQuantas++;	
		}
	}

	return nextTwist;
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

inline bool isVelocityIllegal(double velocity) {
        return (velocity < 0.0 || velocity > 1.0);
}

/* req has two fields req.clockwise and req.fraction_of_max_angular_velocity 
 * clockwise indicates which direction to rotate in
 * fraction of max velocity indicates the percent of the max angular velocity
 * to rotate at. Defaults to one half max velocity if given value is not legal
 *  
*/
bool start_waypoints(nuturtle_robot::start_waypoints::Request  &req, nuturtle_robot::start_waypoints::Response &res) {

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


/* Take the odometry data and update our local values 
 * of where the robot is according to the odometry data
 * Remeber that the odometry data is still just a prediction
 * of where we are. It is not the ground truth 
 */
void odomCallback(const nav_msgs::Odometry odom_data) {
	
	current_odom_x = odom_data.pose.pose.position.x;
	current_odom_y = odom_data.pose.pose.position.y;
	
	// Convert the quaternion to an angle 
	tf::Quaternion q(odom_data.pose.pose.orientation.x,
        		 odom_data.pose.pose.orientation.y,
        		 odom_data.pose.pose.orientation.z,
       			 odom_data.pose.pose.orientation.w);
    
	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;

	// yaw is rotation about the z-axis
    	m.getRPY(roll, pitch, yaw);
    
    	current_odom_theta = yaw;
	//std::cout << "The current odom_theta is " << current_odom_theta << std::endl;
}


/* Add description
*/
int main(int argc, char **argv) {

	ros::init(argc, argv, "waypoint_node");
	ros::NodeHandle n;
	
	//logParams();
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


	int frequency;
	n.getParam("/frequency", frequency);
	// This specifies the rate at which we loop - this means loop at 1000 Hz
	ros::Rate loop_rate(frequency);

	//FSM_Feedback myFSM = FSM_Feedback();
	FSM_Feedforward myFSM = FSM_Feedforward(double(frequency));
	
	turtlesim::Pose poseNow;
	int count = 0;

	while(!hasStarted) {
                ros::spinOnce();
        }
	ros::spinOnce();

	myFSM.setRates();

	while (ros::ok()) {

		geometry_msgs::Twist currentTwist = myFSM.checkUpdate(); 

		cmd_vel_pub.publish(currentTwist);

		ros::spinOnce();

		//loop_rate.sleep();
		count++;
	}

	return 0;
}
