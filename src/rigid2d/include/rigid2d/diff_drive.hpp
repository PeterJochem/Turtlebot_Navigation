/// \file
/// \brief Implements high level abstraction and tracking of a diff drive robot

#include<iosfwd>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Pose2D.h>

namespace rigid2d {

	/// \brief Desribes desired wheel velocities in radians/s
	struct WheelVelocities {
        	double left = 0.0;
        	double right = 0.0;
	};
	
	/// @brief Implements high level abstraction and tracking of a diff drive robot 
	class DiffDrive {
		public:
			/// @brief The default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
			DiffDrive();

			/** @brief Constructor - create a DiffDrive model by specifying the pose, and geometry
			* @param pose - the current position of the robot
			* @param wheel_base - the distance between the wheel centers
			* @param wheel_radius - the radius of the wheels */
			DiffDrive(Transform2D, double wheel_base, double wheel_radius);
			
			/** @brief Compute the robot's encoder values
			 *  @return the robot's current encoder values */
			std::tuple<double, double> getEncoders(void);			
			
			/** @brief Determine the wheel velocities required to make the robot
			*          move with the desired linear and angular velocities
			* @param twist - the desired twist in the body frame of the robot
			* @return - the wheel velocities to use
			* @throws std::exception */
			WheelVelocities twistToWheels(Twist2D);
				
			/** @brief determine the body twist of the robot from its wheel velocities
			*   @param vel - the velocities of the wheels, assumed to be held constant
			*  	         for one time unit
			*   @return the twist in the original body frame of the robot */
			Twist2D wheelsToTwist(WheelVelocities vel);
				
			/** @brief Update the robot's odometry based on the current encoder readings
			*   @param left - the left encoder angle (in radians)
			*   @param right - the right encoder angle (in radians) */
			WheelVelocities updateOdometry(double, double);

			/** @brief update the odometry of the diff drive robot, assuming that
			* 	   it follows the given body twist for one time  unit
			* @param cmd - the twist command to send to the robot */
			void feedforward(Twist2D);

			/** @brief Get the current pose of the robot
			 * @return the current pose of the robot */
			geometry_msgs::Pose2D pose();

			/** @brief reset the robot to the given position/orientation
			* @param ps - the new pose of the robot */
			void reset(Twist2D ps);

			/** @brief Set the robot's pose
			 *  @param x - the new x coordinate
			 *  @param y - the new y coordinate
			 *  @param theta - the robot's new angle in the plane */ 
			void setPose(double x, double y, double theta);

			/* @brief Retreive the robot's pose
			 * @return the robot's pose as a Transform2D */
			Transform2D getPose(void);

			rigid2d::Transform2D current_pose;
			double wheel_base;
			double wheel_radius;
			double encoder_left;
	                double encoder_right;
	};
}
