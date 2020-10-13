/** @file
 * @brief Describe  
 *
 * Parameters: /rotational_vel_limit: max rotational speed
 *             /trans_vel_limit: max translational speed
 *             /k_p_trans: proportional gain for translating
 *             /k_i_trans: integral gain for translating
 *             /k_p_rot: proportional gain for rotating
 *             /k_i_rot: integral gain for rotating
 *             /linear_threshold: linear distance where we consider robot at a point
 *             /angular_threshold: angular distance where we consider robot'a angle to be equal
 *
 *
 * Publishes: /odom: standard ROS odometry message
 *            /visualization_marker: Marker for RVIZ to denote where the robot has been
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
#include <nav_msgs/OccupancyGrid.h>
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
#include <queue>

double max_rotation_speed, max_translational_speed, frac_rot_vel, frac_trans_vel, linear_threshold, angular_threshold;
double angle_integral = 0.0;
double k_p_trans, k_i_trans, k_p_rot, k_i_rot;


// Change the type
std::priority_queue<int> search_queue; // AStar's search queue


void processMap(const nav_msgs::OccupancyGrid map) {
	
	std::cout << std::endl << (int)map.data[0] << std::endl;	

	return;	
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "motion_planning_node");
        ros::NodeHandle n;

        n.getParam("/rotational_vel_limit", max_rotation_speed);
        n.getParam("/trans_vel_limit", max_translational_speed);
        n.getParam("/k_p_trans", k_p_trans);
        n.getParam("/k_i_trans", k_i_trans);
        n.getParam("/k_p_rot", k_p_rot);
        n.getParam("/k_i_rot", k_i_rot);
        n.getParam("/linear_threshold", linear_threshold);
        n.getParam("/angular_threshold", angular_threshold);
	

	ros::Subscriber map_sub = n.subscribe("/map", 1, processMap);
	
	
	while (ros::ok()) {
                //geometry_msgs::Twist currentTwist = myFSM.checkUpdate();
                //cmd_vel_pub.publish(currentTwist);
                ros::spinOnce();
                //count++;
        }

	
	return 0;
}


