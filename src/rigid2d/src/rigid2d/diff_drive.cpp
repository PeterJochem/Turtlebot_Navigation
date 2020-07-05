/// \brief describe this file 

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>

namespace rigid2d {

	/* Create a diff_drive robot at (0, 0, 0) 
         */
       	DiffDrive::DiffDrive() {
	
		this->current_pose = Transform2D();	
	
		// FIX ME 
		// What to set this to?
		// Wheel base is the distance between the center of the wheels
		this->wheel_base = 0.5;
                this->wheel_radius = 0.1;
        }
	
	/* Describe 
	 */
	DiffDrive::DiffDrive(Transform2D pose, double wheel_base, double wheel_radius) {
		
		this->current_pose = pose;
		this->wheel_base = wheel_base;
		this->wheel_radius = wheel_radius;
	}

	/* Map a desired twist in the body frame to the
	 * wheel velocities
	 * The derivation of these equations is at (FIX ME)  
	 */
	WheelVelocities DiffDrive::twistToWheels(Twist2D tw) {
		
		WheelVelocities wheel_vel;
			
		// Wheel base is the distance between the center of the wheels
		wheel_vel.left = (tw.w * (-wheel_base/ (2 * wheel_radius) ) ) + (tw.dx/wheel_radius);       
					
		wheel_vel.right = (tw.w * wheel_base/ (2 * wheel_radius) ) + (tw.dx/wheel_radius);
				
		return wheel_vel;
	}		

		
			
		
		




}
