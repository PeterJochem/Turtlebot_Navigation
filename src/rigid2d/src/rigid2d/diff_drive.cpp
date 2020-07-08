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
	 * FIX ME - add defaults for the wheel base and wheel radius
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
		// what is relationship between wheel base and d in derivation?
		wheel_vel.left = (tw.w * (-wheel_base/ (2 * wheel_radius) ) ) + (tw.dx/wheel_radius);       
					
		wheel_vel.right = (tw.w * wheel_base/ (2 * wheel_radius) ) + (tw.dx/wheel_radius);
				
		return wheel_vel;
	}		
	
	/* Given wheel velocities, 
	 * compute the twist in the body frame
	 * Returns Twist2D in the body frame 
	 */
	Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel) {
		
		double A = 1.0 / (wheel_base);	
		double w = wheel_radius * (-A * vel.left + A * vel.right);	
		
		double dx = wheel_radius * (0.5 * vel.right + 0.5 * vel.left); 
		double dy = 0.0;
			
		Twist2D Twist_left_wheel_frame = Twist2D(w, dx, dy);
	
		// Put in the param server/ make constant in class?
		Vector2D v;
		v.x = 0.0;
		v.y = wheel_base / 2;
		Transform2D T_body_left_wheel = Transform2D(v);
		
		//return Twist_left_wheel_frame;
		return T_body_left_wheel(Twist_left_wheel_frame); 
	}
		
			
		
		




}
