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
                this->tf(0, 1) = -1 * sin(theta);
                this->tf(0, 2) = x;

                this->tf(1, 0) = sin(theta);
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

	/* Print a human readable description of the Transform2D
	 */
	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               // return os << tf.getX() << std::endl;
               return os << "dtheta (degrees): " << rad2deg(tf.getTheta()) << "    dx: " << tf.getX() 
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


	/* Compose two transformations together and return the result
	 * lhs - the left hand side transformation
	 * rhs - the right hand side transformation
	 * Returns the composition of the two transformations
	 */ 
	Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
		
		Eigen::Matrix<float, 3, 3> resultingMatrix = lhs.getTf() * rhs.getTf();    

		double dx = resultingMatrix(0, 2);
		double dy = resultingMatrix(1, 2);
		
		Vector2D vector;
		vector.x = dx;
		vector.y = dy;

		double theta = acos(resultingMatrix(0, 0));
		Transform2D result = Transform2D(vector, theta);  

		return result;
	}

	/* Reads in a Transform2D from the user
	 */	
	std::istream & operator>>(std::istream & is, Transform2D & tf) {
	
		double x;
		double y;
		double angle;

		std::cout << "Enter the angle (degrees) \n";
		is >> angle;
		angle = deg2rad( float(angle) );

		std::cout << "Enter x \n";
		is >> x;

		std::cout << "Enter y \n";
		is >> y;
		
		tf.setMatrices(x, y, angle);

		return is;
	}

	/* Describe this method 
	 */
	Twist2D Transform2D::operator()(Twist2D twist, Transform2D pTF) {
		
		// Twist_newFrame = [Adjoint_T] * Twist_original_frame
		// See page 85 of Modern Robotics
			// [p] is the skew shymmetric matrix of the three vector
			// see page 65 of Modern Robotics  

		// Create the adjoint	
		// Use the adjoint to map into the new coordinate frame
			
		//

		// w first?
		return rigid2d::Twist2D(1, 2, 3);
	}


	// End of Transform2D class 
	
	// Start of the Twist2D class 	
	Twist2D::Twist2D(double w, double dx, double dy) {
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/* Create the (0.0, 0.0, 0.0) twist
	 */
	Twist2D::Twist2D(void) {
                this->w = 0.0;
                this->dx = 0.0;
                this->dy = 0.0;
        }
	
	void Twist2D::setVars(double w, double dx, double dy) {
		
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/* Print a human readable description of the twist
	 */	
	std::ostream & operator<<(std::ostream & os, const Twist2D &tw) {
	
		std::cout << "Angular part: " << tw.w << "     dx: " << tw.dx << "    dy: " << tw.dy << std::endl;

		return os;
	}		

	/* Read in a Twist2d from the user
	 */
	std::istream & operator>>(std::istream & is, Twist2D &tw) {
	
		double w;
		double dx;
		double dy;

		std::cout << "Enter the angular part (rad/s) ";
		is >> w;

		std::cout << "Enter dx ";
                is >> dx;

		std::cout << "Enter dy ";
                is >> dy;		
		
		tw.setVars(w, dx, dy);		

		return is;
	}





}
