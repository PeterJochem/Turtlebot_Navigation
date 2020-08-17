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
        
		this->encoder_left = 0.0;
		this->encoder_right = 0.0;
	}
	
	/* Describe 
	 * FIX ME - add defaults for the wheel base and wheel radius
	 */
	DiffDrive::DiffDrive(Transform2D pose, double wheel_base, double wheel_radius) {
		
		this->current_pose = pose;
		this->wheel_base = wheel_base;
		this->wheel_radius = wheel_radius;
		
		this->encoder_left = 0.0;
		this->encoder_right = 0.0;
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
					
		double A = 1.0 / wheel_base;	
		double w = wheel_radius * (-A * vel.left + A * vel.right);	
		
		double dx = wheel_radius * (0.5 * vel.right + 0.5 * vel.left); 
		double dy = 0.0;
		
		return Twist2D(w, dx, dy);		
	}
	
	/* Update the robot's odometry given new encoder angles
	 * left - the left encoder angle (in radians)
    	 * right - the right encoder angle (in radians)
	 */	
	WheelVelocities DiffDrive::updateOdometry(double left, double right) { 
		
		WheelVelocities vel;
                vel.left = left - encoder_left;
                vel.right = right - encoder_right;
			
		// Update the robot's encoder fields
		encoder_left = left;
		encoder_right = right;
		
		// Compute the twist of the robot's BODY frame
		Twist2D Vb = wheelsToTwist(vel);
		current_pose = current_pose.integrateTwist(Vb);
	
		return {encoder_left, encoder_right};
	}
			
	/* Given a twist, update current_pose and the encoder angles
	 * of the robot assuming the twist was followed for 1 unit 
	 * of time
	 * Twist2D tw is the twist the robot follows for unit time
	 */
	void DiffDrive::feedforward(Twist2D tw) { 
		
		WheelVelocities vel = twistToWheels(tw);

		// Update encoder angles
		encoder_left += vel.left;
                encoder_right += vel.right;

		// Update current_pose		
		current_pose = current_pose.integrateTwist(tw); 	 
	}
	
	/* Return the robot's current pose as a geometry_msgs
	 * Pose2D  
	 */	
	geometry_msgs::Pose2D DiffDrive::pose() {

		geometry_msgs::Pose2D pose;
		pose.x = current_pose.getX();
		pose.y = current_pose.getY();	
		pose.theta = normalize_angle( current_pose.getTheta() );

		return pose;
	}	
		
	/* Get method for the encoder values
	 */
	std::tuple<double, double> DiffDrive::getEncoders(void) {
		
		return {encoder_left, encoder_right};
	}


}
