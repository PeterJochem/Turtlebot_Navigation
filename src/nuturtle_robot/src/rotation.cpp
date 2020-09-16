/** @file
 *  @brief Has robot do 20 rotations in the real world. Each rotation is followd by a short pause 
 *  
 *  Parameters: /rotational_vel_limit: Maximum speed the robot can rotate at
 * 
 *  Publishes: /turtle1/cmd_vel: Twist for the robot to follow
 *
 *  Subscribes: None 
 *
 *  Services: /start: Start the robot rotating */

#include <sstream>
#include <stdlib.h>
#include <cmath> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "nuturtle_robot/start.h"
#include "rigid2d/setPose.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

bool rotate = false;
bool hasStarted = false;
double rotation_speed = 0.0;
int timerEventCount = 0;
double publishRate, timeTo360, max_rotation_speed;
int numRotations = 0;
ros::ServiceClient set_pose_client;
ros::Publisher cmd_vel_pub;

/** @brief Check if user defined velocity is legal
 *  @param velocity is the fraction of the maximum velocity the robot moves at
 *  @return true if velocity is within 0 and 1, false otherwise */
inline bool isVelocityIllegal(double velocity) {
	return (velocity < 0.0 || velocity > 1.0);		
}

/** @brief Check if we shoud top rotating 
 *  @return true if we rotated for as long as scheduled rotation period is, false otherwise */
inline bool stopRotating() {
	return (rotate && std::abs(timerEventCount * publishRate) >= timeTo360); 
}

/** @brief Check if we should make robot rotate 
 *  @return true if it is time to rotate, false otherwise */
inline bool startRotating(double pausePeriod = timeTo360/10.0) {
	return !rotate && std::abs(timerEventCount * publishRate) >= pausePeriod; 
}

/** @brief Start the robot rotating  
 *  @param req - Two fields req.clockwise and req.fraction_of_max_angular_velocity 
 * 	   clockwise indicates which direction to rotate in
 * 	   fraction of max velocity indicates the percent of the max angular velocity
 * 	   to rotate at. Defaults to one half max velocity if given value is not legal
 *  @param res - I do not use but is required for ROS.
 *  @return True */
bool start(nuturtle_robot::start::Request  &req, nuturtle_robot::start::Response &res) {
	
	rigid2d::setPose newPose;
	newPose.request.x = 0.0;
	newPose.request.y = 0.0;
	newPose.request.theta = 0.0;
	set_pose_client.call(newPose);	
	
	if (isVelocityIllegal(req.fraction_of_max_angular_velocity) ) {
		rotation_speed = 0.5 * max_rotation_speed;	
	}
	else if (req.clockwise) {
		rotation_speed = req.fraction_of_max_angular_velocity * max_rotation_speed;
	}
	else {
		rotation_speed = -1 * req.fraction_of_max_angular_velocity * max_rotation_speed;	
	}
	
	numRotations = 0;
	rotate = true;
	hasStarted = true;
	return true;
}

/** @brief Timer callback which publishes the next 
 * 	   twist for the robot to follow 
 *  @param TimerEvent is ROS object */
void publishNextTwist(const ros::TimerEvent&) {
	
	geometry_msgs::Twist newTwist;	
	timerEventCount++;	
	
	if (numRotations > 20) {
		ros::shutdown();
	}

	if (stopRotating()) {
		timerEventCount = 0;	
		newTwist.angular.z = 0.0;
		rotate = false;
		numRotations++;
		std::cout << "Completed Rotation " << numRotations << std::endl;
	}
	
	else if (startRotating()) {
		timerEventCount = 0;
		newTwist.angular.z = rotation_speed;
		rotate = true;
	}

	if (rotate) {
		newTwist.angular.z = rotation_speed;
	}

	cmd_vel_pub.publish(newTwist);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "rotation");
	ros::NodeHandle n;

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);	
	n.getParam("/rotational_vel_limit", max_rotation_speed);	

	ros::service::waitForService("/set_pose", -1);
        ros::ServiceClient set_pose_client = n.serviceClient<rigid2d::setPose>("/set_pose");	
	ros::ServiceServer start_service = n.advertiseService("start", start);	
	
	while(!hasStarted) {
		ros::spinOnce();
	}
		
	timeTo360 = std::abs((2 * rigid2d::PI) / rotation_speed);
	publishRate = 1.0/100.0;
	 
	ros::Timer timer = n.createTimer(ros::Duration(publishRate), publishNextTwist);	
	ros::spin();			

	return 0;
}
