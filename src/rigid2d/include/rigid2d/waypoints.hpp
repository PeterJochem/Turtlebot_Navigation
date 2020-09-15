/// @file
/// @brief Implements methods for navigating to a series of waypoints

#include<iosfwd> 
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Pose2D.h>

namespace rigid2d {

	/// @brief implements methods for navigating to a series of waypoints via a rotate and then translate strategy.
	class WayPoints {
		public:
			/// @brief The Default constructor. Creates a robot at (0,0,0), with an arbitrary wheel base and wheel radius
			WayPoints(void);
			
			/** @brief Constructor 
			 *  @param points - a list of waypoints to navigate to */
			WayPoints(std::vector<Vector2D> points);

			/** @brief Uses the given robot's SE(2) orientation
                         * 	   to compute what the next waypoint is and if we 
   		         * 	   should rotate or translate. Constrains the robot
        		 *         to a rotate and then translate strategy.  
			 * @param current_SE2 is the robot's current transform
			 * @return Twist2D - the twist that the robot should follow */
			Twist2D nextWayPoint(Transform2D current_SE2);
			
			std::vector<Vector2D> points;
			int index; // position in the list of waypoints
	};
}



