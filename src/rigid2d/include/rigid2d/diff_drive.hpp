/// \file
/// \brief Describe this file

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Pose2D.h>

/// \brief Desribes desired wheel velocities 
// in radians/s

namespace rigid2d {

	/// \brief Desribes desired wheel velocities 
	// in radians/s
	struct WheelVelocities {
        	double left = 0.0;
        	double right = 0.0;
	};

	
	class DiffDrive {
		public:
			/// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
			DiffDrive();

			/// \brief create a DiffDrive model by specifying the pose, and geometry
			///
			/// \param pose - the current position of the robot
			/// \param wheel_base - the distance between the wheel centers
			/// \param wheel_radius - the radius of the wheels
			DiffDrive(Transform2D, double, double);
							

			/// \brief determine the wheel velocities required to make the robot
			///        move with the desired linear and angular velocities
			/// \param twist - the desired twist in the body frame of the robot
			/// \returns - the wheel velocities to use
			/// \throws std::exception
			WheelVelocities twistToWheels(Twist2D);
				
			/// \brief determine the body twist of the robot from its wheel velocities
			/// \param vel - the velocities of the wheels, assumed to be held constant
			///  for one time unit
			/// \returns twist in the original body frame of the
			Twist2D wheelsToTwist(WheelVelocities vel);
				
			/// \brief Update the robot's odometry based on the current encoder readings
			/// \param left - the left encoder angle (in radians)
			/// \param right - the right encoder angle (in radians)
			WheelVelocities updateOdometry(double, double);

			/// \brief update the odometry of the diff drive robot, assuming that
			/// it follows the given body twist for one time  unit
			/// \param cmd - the twist command to send to the robot
			void feedforward(Twist2D);

			/// \brief get the current pose of the robot
			geometry_msgs::Pose2D pose();

			/// \brief get the wheel speeds, based on the last encoder update
			/// \returns the velocity of the wheels, which is equivalent to
			/// displacement because \Delta T = 1
			//WheelVelocities wheelVelocities() const

			/// \brief reset the robot to the given position/orientation
			//void reset(Twist2D ps);

			// Store a Transform2D
			rigid2d::Transform2D current_pose;

			double wheel_base;
			double wheel_radius;

			double encoder_left;
	                double encoder_right;
	};

}



