/* File description
 */
#include "rigid2d.hpp"
#include <iostream>
#include <Eigen/Dense>

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
		
		// Create the transformation matrix
		// Eigen uses column major order
		this->tf(0, 0) = cos(theta);	
		this->tf(0, 1) = sin(theta);
		this->tf(0, 2) = x;
		
		this->tf(1, 0) = -1 * sin(theta);
		this->tf(1, 1) = cos(theta);
		this->tf(1, 2) = y;
		
		this->tf(2, 0) = 0.0;
		this->tf(2, 1) = 0.0;
		this->tf(2, 2) = 1.0;
	}

	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               // return os << tf.getX() << std::endl;
               return os << "dtheta (degrees): " << tf.getTheta() << "    dx: " << tf.getX() 
		       << "    dy: " << tf.getY() << std::endl;
	}
	
	double Transform2D::getX(void) const {
		return x;
	}

	double Transform2D::getY(void) const {
                return y;
        }

	double Transform2D::getTheta(void) const {
                return theta;
        }

	Eigen::Matrix<float, 3, 3> Transform2D::getTf(void) const {
                return tf;
        }


}
