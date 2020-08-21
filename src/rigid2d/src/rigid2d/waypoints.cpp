/// \brief describe this file 

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>

namespace rigid2d {

	/* Constructor
	 */
	WayPoints::WayPoints(std::vector<Vector2D> points) {
		// FIX ME - if length of list is 0, add the (0, 0) point
		this->points = points;
		this->index = 0;
	}
	
	/* Describe 
	 */
	Twist2D WayPoints::nextWayPoint(Transform2D T_world_robot) {
		
		Vector2D current_point = points[index];

		double currentAngle = T_world_robot.getTheta(); 
                double desiredAngle = atan2(current_point.y - T_world_robot.getY(), current_point.x - T_world_robot.getX());

                double angleError = desiredAngle - currentAngle;
                double translationalError = sqrt(pow(current_point.y - T_world_robot.getY(), 2) + pow(current_point.x - T_world_robot.getX(), 2) );	
		// Check for reaching of point 
		if ( (almost_equal(angleError, 0.0) && (almost_equal(translationalError, 0.0) ) ) ) {
                        index = index + 1;
                        if (index >= int(points.size() ) ) {
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
					
		return this->computeNextWayPoint(T_world_robot, current_point);
	}


	/* Describe
	 * Given the robot's position in the world frame, and the
	 * ...
	 * Compute the next Twist2D 
	 */
	Twist2D WayPoints::computeNextWayPoint(Transform2D T_world_robot, Vector2D point) {
				
		double currentAngle = T_world_robot.getTheta();	
		double desiredAngle = atan2(point.y - T_world_robot.getY(), point.x - T_world_robot.getX());
			
		double angleError = desiredAngle - currentAngle;
		double translationalError = sqrt(pow(point.y - T_world_robot.getY(), 2) + pow(point.x - T_world_robot.getX(), 2) );	
		
		if (almost_equal(angleError, 0.0)) {
			// Translate
			
			// In the robot's frame, "forward" is the +x axis
                        return Twist2D(0.0, translationalError, 0.0);
		}
		else {
			// rotate 
			// FIX ME - max translational velocity?
				 
			// FIX ME - max angular velocity?
                        return Twist2D(angleError, 0.0, 0.0);
		}		
	}	

}
