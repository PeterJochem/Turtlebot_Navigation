/** @file
 *  @brief Has robot do a few 1 meter translations in the real world. Each translation is followd by a short pause 
 *  
 *  Parameters: /trans_vel_limit: Maximum speed the robot can translate at
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

bool translate = false;
bool hasStarted = false;
double trans_speed = 0.0;
double segment_length = 1.0; // Length of each segment 
int timerEventCount = 0;
double publishRate, timeToTranslate, max_translational_speed;
int numSegments = 0;
ros::ServiceClient set_pose_client;
ros::Publisher cmd_vel_pub;


/** @brief Check if user defined velocity is legal
 *  @param velocity is the fraction of the maximum velocity the robot moves at
 *  @return true if velocity is within 0 and 1, false otherwise */
inline bool isVelocityIllegal(double velocity) {
	return (velocity < 0.0 || velocity > 1.0);		
}

/** @brief Check if we shoud top translating 
 *  @return true if we translated for as long as scheduled translation period is, false otherwise */
inline bool stopTranslating() {
	return (translate && std::abs(timerEventCount * publishRate) >= timeToTranslate); 
}

/** @brief Check if we should make robot translate 
 *  @return true if it is time to translate, false otherwise */
inline bool startTranslating(double pausePeriod = timeToTranslate/10.0) {
	return !translate && std::abs(timerEventCount * publishRate) >= pausePeriod; 
}

/** @brief Start the robot rotating  
 *  @param req - Two fields req.clockwise and req.fraction_of_max_angular_velocity 
 *         clockwise indicates which direction to rotate in
 *         fraction of max velocity indicates the percent of the max angular velocity
 *         to rotate at. Defaults to one half max velocity if given value is not legal
 *  @param res - I do not use but is required for ROS.
 *  @return True */
bool start(nuturtle_robot::start::Request  &req, nuturtle_robot::start::Response &res) {
	
	rigid2d::setPose newPose;
	newPose.request.x = 0.0;
	newPose.request.y = 0.0;
	newPose.request.theta = 0.0;
	set_pose_client.call(newPose);	
	
	if (isVelocityIllegal(req.fraction_of_max_angular_velocity) ) {
		trans_speed = 0.5 * max_translational_speed;	
	}
	else if (req.clockwise) {
		trans_speed = req.fraction_of_max_angular_velocity * max_translational_speed;
	}
	else {
		trans_speed = -1 * req.fraction_of_max_angular_velocity * max_translational_speed;	
	}
	
	numSegments = 0;
	translate = true;
	hasStarted = true;
	return true;
}

/** @brief Timer callback which publishes the next 
 *         twist for the robot to follow 
 *  @param TimerEvent is ROS object */
void publishNextTwist(const ros::TimerEvent&) {
	
	geometry_msgs::Twist newTwist;	
	timerEventCount++;	
	
	if (numSegments > 20) {
		ros::shutdown();
	}

	if (stopTranslating()) {
		timerEventCount = 0;	
		newTwist.linear.x = 0.0;
		translate = false;
		numSegments++;
		std::cout << "Completed Segment " << numSegments << std::endl;
	}
	
	else if (startTranslating()) {
		timerEventCount = 0;
		newTwist.linear.x = trans_speed;
		translate = true;
	}

	if (translate) {
		newTwist.linear.x = trans_speed;
	}

	cmd_vel_pub.publish(newTwist);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "translate");
	ros::NodeHandle n;

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
	
	n.getParam("/trans_vel_limit", max_translational_speed);	

	ros::service::waitForService("/set_pose", -1);
        ros::ServiceClient set_pose_client = n.serviceClient<rigid2d::setPose>("/set_pose");
		
	ros::ServiceServer start_service = n.advertiseService("start", start);	
	
	while(!hasStarted) {
		ros::spinOnce();
	}
		
	timeToTranslate = std::abs(segment_length / trans_speed);
	publishRate = 1.0/100.0;
	
	ros::Timer timer = n.createTimer(ros::Duration(publishRate), publishNextTwist);	
	ros::spin();			

	return 0;
}
