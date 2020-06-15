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
	
	 
	/* Describe this method
         */
        void Transform2D::setMatrices(double x, double y, double theta) {

                this->vector.x = x;
                this->vector.y = y;
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

	/* Create the identity transformation 
	 */ 	
	Transform2D::Transform2D (void) {
		
		this->setMatrices(0.0, 0.0, 0.0);
	}

	/* Create a transformation which has both a rotation and 
	 * a translation
	 * trans - the vector describing the translation
	 * radians - the angle of rotation
	 */ 
	Transform2D::Transform2D(const Vector2D & trans, double radians) {

		this->setMatrices(trans.x, trans.y, radians);		
	}


	/* Create a transformation that is just a transformation
	 */
	Transform2D::Transform2D(const Vector2D & trans) {
		
		this->setMatrices(trans.x, trans.y, 0.0);
	}
	
	/* Create a Transform2D which is just a rotation
	 * Radians - the angle of rotation about the z-axis
	 */
	Transform2D::Transform2D(double radians) {
		
		this->setMatrices(0.0, 0.0, radians);	
	}


	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               // return os << tf.getX() << std::endl;
               return os << "dtheta (degrees): " << tf.getTheta() << "    dx: " << tf.getX() 
		       << "    dy: " << tf.getY() << std::endl;
	}
	
	double Transform2D::getX(void) const {
		return vector.x;
	}

	double Transform2D::getY(void) const {
                return vector.y;
        }

	double Transform2D::getTheta(void) const {
                return theta;
        }

	Eigen::Matrix<float, 3, 3> Transform2D::getTf(void) const {
                return tf;
        }
	
	/* Describe this method 
	 */	
	Transform2D Transform2D::inv() const {

		Transform2D newTransform = Transform2D();	
				
		newTransform.tf = this->tf.inverse();   

		newTransform.vector.x = newTransform.tf(0, 2);
		newTransform.vector.y = newTransform.tf(1, 2);
		newTransform.theta = acos(newTransform.tf(0, 0));

		return newTransform;
	}


	/* Describe this method
	 */
	Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
				
		tf = tf * rhs.tf;

		// Should I force certain columns to be 0? Numerical error?
		vector.x = tf(0, 2);
		vector.y = tf(1, 2);
		
		// Numerical error?
		theta = acos(tf(0, 0)); 

		return *this;
	}
	
	/* Apply the transformation to the vector v
	 * Vector2D is the vector we are applying the transformation to 
	 * Return the resulting Vector2D
	 */
	Vector2D Transform2D::operator()(Vector2D v) const {

		Vector2D newVector;
		
		Eigen::Matrix<float, 3, 1> rhs;
		rhs(0, 0) = v.x;   
		rhs(1, 0) = v.y;
		rhs(2, 0) = 1.0;

		Eigen::Matrix<float, 3, 1> resultingMatrix = this->tf * rhs;  		
		
		newVector.x = resultingMatrix(0, 0);
		newVector.y = resultingMatrix(1, 0);

		return newVector;
	}


}
