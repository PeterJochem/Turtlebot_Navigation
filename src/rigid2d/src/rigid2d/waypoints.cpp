/*! \file
* \brief Waypoints.cpp implements methods for navigating to a series of waypoints. The robot adopts a 
* rotate and translate strategy. I only used
* this for testing code in the turtlesim environment.
*
* PARAMETERS:
*     None
* PUBLISHES:
*     None
* SUBSCRIBES:
*     None
* SERVICES:
* 	None
!*/

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>

namespace rigid2d {
	
       	/*! \brief Default constructor. If there are waypoints, just have the robot stay at (0, 0)
	* so add the point (0, 0) to the list of waypoints.
	!*/
	WayPoints::WayPoints() {
                this->points = std::vector<Vector2D>();
		Vector2D v;
		v.x = 0.0;
		v.y = 0.0;
                this->points.push_back(v);
		this->index = 0;
        }


	/*! \brief Default constructor. If there are waypoints, just have the robot stay at (0, 0)
        * so add the point (0, 0) to the list of waypoints.
	* \tparam points - the list of points to navigate to
	!*/
	WayPoints::WayPoints(std::vector<Vector2D> points) {
		// FIX ME - if length of list is 0, add the (0, 0) point
		this->points = points;
		this->index = 0;
	}
	
	/*! \brief Uses the given robot's SE(2) orientation
	* to compute what the next waypoint is and if we 
	* should rotate or translate. Constrains the robot
	* to a rotate and then translate strategy.  
	* 
	* \param T_world_robot - the robot's SE(2) orientation
	* \return Twist2D - the twist that the robot should follow
	!*/
	Twist2D WayPoints::nextWayPoint(Transform2D T_world_robot) {
		
		Vector2D current_point = points[index];

		double currentAngle = T_world_robot.getTheta(); 
                double desiredAngle = atan2(current_point.y - T_world_robot.getY(), current_point.x - T_world_robot.getX());

                double angleError = desiredAngle - currentAngle;
                double translationalError = sqrt(pow(current_point.y - T_world_robot.getY(), 2) + pow(current_point.x - T_world_robot.getX(), 2) );	
		
		// Check for reaching of point 
		if ((almost_equal(angleError, 0.0) && (almost_equal(translationalError, 0.0)))) {
                        index = index + 1;
                        if (index >= int(points.size())) {
                          	std::cout << "Reached the end of the list of waypoints. Looping back" << std::endl;
			  	index = 0;
			}

		       	// Compute the data for the new, next waypoint	
			current_point = points[index];

                	currentAngle = T_world_robot.getTheta();
                	desiredAngle = atan2(current_point.y - T_world_robot.getY(), current_point.x - T_world_robot.getX());

                	angleError = desiredAngle - currentAngle;
                	translationalError = sqrt(pow(current_point.y - T_world_robot.getY(), 2) + pow(current_point.x - T_world_robot.getX(), 2)); 
		}
		
		Twist2D nextTwist;		
		if (almost_equal(angleError, 0.0)) {
                        // Translate - In the robot's frame, "forward" is the +x axis
                        nextTwist = Twist2D(0.0, translationalError, 0.0);
                }
                else {
                        // Rotate 
                        nextTwist = Twist2D(angleError, 0.0, 0.0);
                }
		
		return nextTwist;
	}
}
