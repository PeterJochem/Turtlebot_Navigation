/* File description
 */
#include "rigid2d.hpp"
#include <iostream>
#include <Eigen/Dense>
// #include "/usr/include/boost/numeric/odeint/external/eigen/eigen.hpp"

namespace rigid2d {
	
	/* Describe this method 
	 */
	std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
		return os << "[" << v.x << ", " << v.y <<  "]" << std::endl;  
	}

	/* Describe this method	
	std::istream & operator>>(std::istream & is, Vector2D & v) {

	}
	*/


	/* This is the constructor for the Transform 2d
	 */
	Transform2D::Transform2D (double x, double y, double theta) {
		this->x = x; 
		this->y = y;
		this->theta = theta; 		
	}

	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               // return os << tf.getX() << std::endl;
               return os << 100;
	}
	
	double Transform2D::getX(void) {
		return this->x;
	}

}
