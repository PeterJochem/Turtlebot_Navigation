/// @brief Implements high level abstraction and tracking of a differential drive robot 

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>

namespace rigid2d {

	/** @brief Default Constructor - Creates a diff_drive robot at (0, 0, 0) */
       	DiffDrive::DiffDrive() {
	
		this->current_pose = Transform2D();	
	
		// Arbitrary wheel_base and wheel_radius
		this->wheel_base = 0.5;
                this->wheel_radius = 0.1;
        
		this->encoder_left = 0.0;
		this->encoder_right = 0.0;
	}
	
	/** @brief Constructor
	 * @param pose - the robot's SE(2) configiration
	 * @param wheel_base - half the distance between the wheels
	 * @param wheel_radius - the wheel's radius */
	DiffDrive::DiffDrive(Transform2D pose, double wheel_base, double wheel_radius) {
		
		this->current_pose = pose;
		this->wheel_base = wheel_base;
		this->wheel_radius = wheel_radius;
		
		this->encoder_left = 0.0;
		this->encoder_right = 0.0;
	}

	/** @brief Map a desired twist in the body frame to the wheel velocities
	 * @param tw - the twist to map down to the wheel commands
	 * @returns the wheel velocities
	 * The derivation of these equations is in the README.md */
	WheelVelocities DiffDrive::twistToWheels(Twist2D tw) {
		
		WheelVelocities wheel_vel;
		wheel_vel.left = (tw.w * (-wheel_base/ (2 * wheel_radius) ) ) + (tw.dx/wheel_radius);       
		wheel_vel.right = (tw.w * wheel_base/ (2 * wheel_radius) ) + (tw.dx/wheel_radius);
		return wheel_vel;
	}		
	
	/** @brief Given wheel velocities, compute the twist in the body frame
	 *  @returns Twist2D in the body frame */
	Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel) {
					
		double A = 1.0 / wheel_base;	
		double w = wheel_radius * (-A * vel.left + A * vel.right);	
		
		double dx = wheel_radius * (0.5 * vel.right + 0.5 * vel.left); 
		double dy = 0.0;
		
		return Twist2D(w, dx, dy);		
	}
	
	/** @brief Update the robot's odometry given new encoder angles
	 *  @param left - the left encoder angle (in radians)
    	 *  @param right - the right encoder angle (in radians) 
	 *  @return the wheel displacements */	
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

		// Changed on August 31, 2020
		// return {encoder_left, encoder_right};
		return {vel.left, vel.right};
	}
			
	/** @brief Given a twist, update current_pose and the encoder angles
	 * 	   of the robot assuming the twist was followed for 1 unit of time
	 *  @param tw is the twist the robot follows for one unit time */
	void DiffDrive::feedforward(Twist2D tw) { 
		
		WheelVelocities vel = twistToWheels(tw);

		// Update encoder angles
		encoder_left += vel.left;
                encoder_right += vel.right;

		// Update current_pose		
		current_pose = current_pose.integrateTwist(tw); 	 
	}
	
	/** @brief Compute the robot's current pose as a geometry_msgs Pose2D  
	 *  @return the pose as geometry_msgs::Pose type */	
	geometry_msgs::Pose2D DiffDrive::pose() {

		geometry_msgs::Pose2D pose;
		pose.x = current_pose.getX();
		pose.y = current_pose.getY();	
		pose.theta = normalize_angle( current_pose.getTheta() );

		return pose;
	}	
		
	/** @brief Get method for the encoder values
	 *  @return The two encoder values */
	std::tuple<double, double> DiffDrive::getEncoders(void) {	
		return {encoder_left, encoder_right};
	}
	
	/** @brief Set the robot's SE(2) pose
	 *  @param x - the robot's x position
	 *  @param y - the robot's y position
	 *  @param theta - the robot's angle about the z-axis */	
	void DiffDrive::setPose(double x, double y, double theta) {
		
		Vector2D p;
		p.x = x;
		p.y = y;
		current_pose = Transform2D(p, theta);		
	}	
}
