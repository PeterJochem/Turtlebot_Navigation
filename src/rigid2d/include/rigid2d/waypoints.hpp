/// \file
/// \brief Describe this file

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Pose2D.h>


namespace rigid2d {
	class WayPoints {
		public:
			/// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
			WayPoints(std::vector<Vector2D>);

			Twist2D nextWayPoint(Transform2D);
			
			Twist2D computeNextWayPoint(Transform2D T_world_robot, Vector2D point);

			// Fix me
			std::vector<Vector2D> points;
			int index;
	};
}



