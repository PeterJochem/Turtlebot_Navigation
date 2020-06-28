/* File description
 */
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <math.h>
//#include <Eigen/Dense>

namespace rigid2d {
	
	/* Print a vector2D in a human readable format
	 * os - the output stream
	 * v - the vector  
	 */
	std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
		return os << "[" << v.x << ", " << v.y <<  "]" << std::endl;  
	}

	/* Constructor that creates the identity transformation 
	 */ 	
	Transform2D::Transform2D (void) {
		
		cTheta = cos(0.0);
		sTheta = sin(0.0);
		vector.x = 0.0;
		vector.y = 0.0;
	}

	/* Create a transformation which has both a rotation and 
	 * a translation
	 * trans - the vector describing the translation
	 * radians - the angle of rotation
	 */ 
	Transform2D::Transform2D(const Vector2D & trans, double radians) {

		cTheta = cos(radians);
                sTheta = sin(radians);
                vector.x = trans.x;
                vector.y = trans.y;		
	}


	/* Create a transformation that is only a 
	 * transformation without any rotation
	 * p - the vector describing the frame's 
	 * displacement
	 */
	Transform2D::Transform2D(const Vector2D &p) {
		
		cTheta = cos(0.0);
                sTheta = sin(0.0);
                vector.x = p.x;
                vector.y = p.y;
	}
	
	/* Constructor for Transform2D which creates a transform
	 * which is just a rotation
	 * Radians - the angle of rotation about the z-axis
	 */
	Transform2D::Transform2D(double radians) {
		
		cTheta = cos(radians);
                sTheta = sin(radians);
                vector.x = 0.0;
                vector.y = 0.0; 
	}
 
	/* Constructor (one of a few) for the Transform2D class
	 * p - the vector describing the frames translation
	 * cTheta - the cos of the Transform2D's angle about z axis
	 * sTheta - the sin of the Transform2D's angle about the z axis
	 */
	Transform2D::Transform2D(const Vector2D & p, double cTheta, double sTheta) {
		
		this->cTheta = cTheta;
                this->sTheta = sTheta;
                this->vector.x = p.x;
                this->vector.y = p.y; 	
	}	

	/* Print a human readable description of the Transform2D
	 * os - the output stream
	 * tf - the Transform2D we are describing 
	 */
	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               return os << "\ndtheta (degrees): " << rad2deg(tf.getTheta()) << "    dx: " << tf.getX() 
		       << "    dy: " << tf.getY() << std::endl;
	}
	
	// Get methods for Transform2D class	
	double Transform2D::getX(void) const {
		return vector.x;
	}

	double Transform2D::getY(void) const {
                return vector.y;
        }
	
	/* Uses the stored cos(theta) and sin(theta) fields
	 * to recover theta. Returns theta in radians
	 */
	double Transform2D::getTheta(void) const {
                
		// Returns the principal value of the arc tangent of y/x, expressed in radians.
		// To compute the value, the function takes into account the sign of both 
		// arguments in order to determine the quadrant.
		// Note - the call is  double atan2(double y, double x);
		// Principal arc tangent of y/x, in the interval [-pi,+pi] radians
		return std::atan2(sTheta, cTheta);	
	}

	double Transform2D::getCTheta(void) const {
		return cTheta;
	}

	double Transform2D::getSTheta(void) const {
                return sTheta;
        }



	// Get methods for Twist2D class 
	double Twist2D::getDx(void) const {
                return dx;
        }
	
	double Twist2D::getDy(void) const {
                return dy;
        }

	double Twist2D::getW(void) const {
                return w;
        }

	/* Normalizes a given Vector 2D and returns
	 * the normalized vector2D
	 */
	Vector2D normalize(Vector2D orig_vector) {
	
		double magnitude = sqrt(pow(orig_vector.x, 2) + pow(orig_vector.y, 2));
	
		Vector2D new_vector;
		new_vector.x = (orig_vector.x)/magnitude;
		new_vector.y = (orig_vector.y)/magnitude;

		return new_vector;
	}

	/* Comptes and returns the inverse of the given SE(2) matrix
	 * Instead of constructing the entire matrix in Eigen and 
	 * inverting, I opt to use the properties of SE(2) matrices.
	 * This should help reduce the amount of numerical error 
	 * See Modern Robotics chapter 3 for properties of SE(2) matrices 
	 */	
	Transform2D Transform2D::inv() const {

		double x_inv = (-1 * cTheta * vector.x) - (sTheta * vector.y);
		double y_inv = (sTheta * vector.x) - (cTheta * vector.y);
		
		rigid2d::Vector2D vector_inv;
		vector_inv.x = x_inv;
		vector_inv.y = y_inv;

		// Theta should be in radians
		double cTheta_inv = cTheta;
		double sTheta_inv = -1 * sTheta;

		return Transform2D(vector_inv, cTheta_inv, sTheta_inv);
	}

	/* Updates current Transform2D by multiplying it by another Transform2D
	 * rhs - the Transform2D we are composing the current Transform2D with
	 * Return the updated Transform2D 
	 */
	Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
			
		// Compute the new, resulting values of applying the transformation	
		double resultCTheta = (cTheta * rhs.cTheta) + (-1 * sTheta)*(rhs.sTheta);

                double resultSTheta = (sTheta * rhs.cTheta) + (cTheta * rhs.sTheta);

                double dx = (cTheta * rhs.vector.x) + (-1 * sTheta * rhs.vector.y) + (vector.x);

                double dy = (sTheta * rhs.vector.x) + (cTheta * rhs.vector.y) + (vector.y);
		
			
		// Update the fields of the this Transform2D
		cTheta = resultCTheta;
		sTheta = resultSTheta;
		vector.x = dx;
		vector.y = dy;	

		return *this;
	}
	
	/* Apply the transformation to the vector v
	 * Vector2D - the vector we are applying the transformation to 
	 * Return the resulting Vector2D
	 */
	Vector2D Transform2D::operator()(Vector2D v) const {

		Vector2D newVector;
		
		newVector.x = (cTheta * v.x) + (-1 * sTheta * v.y) + vector.x; 
		
		newVector.y = (sTheta * v.x) + (cTheta * v.y) + vector.y;
		 
		return newVector;
	}


	/* Compose two transformations together and return the result
	 * lhs - the left hand side transformation
	 * rhs - the right hand side transformation
	 * Note: A 2D transformation matrix can be stored with only cTheta, sTheta,
	 * x, and y. There's no need to multiply the entire matrix or store the
	 * entire matrix
	 * Returns the composition of the two transformations
	 */ 
	Transform2D operator*(const Transform2D &lhs, const Transform2D &rhs) {
		
		double resultCTheta = (lhs.cTheta * rhs.cTheta) + (-1 * lhs.sTheta)*(rhs.sTheta); 
		
		double resultSTheta = (lhs.sTheta * rhs.cTheta) + (lhs.cTheta * rhs.sTheta);

		double x_new = (lhs.cTheta * rhs.vector.x) + (-1 * lhs.sTheta * rhs.vector.y) + (lhs.vector.x); 
		
		double y_new = (lhs.sTheta * rhs.vector.x) + (lhs.cTheta * rhs.vector.y) + (lhs.vector.y);  
		
		Vector2D resultVector;
		resultVector.x = x_new;
		resultVector.y = y_new;

		return Transform2D(resultVector, resultCTheta, resultSTheta); 
	}

	/* Reads in a Transform2D from the user
	 * is - the input stream
	 * tf - the Transform2D object we are updating/setting fields of
	 */	
	std::istream & operator>>(std::istream & is, Transform2D & tf) {
	
		double angle;

		std::cout << "Enter the angle (degrees) \n";
		is >> angle;
		angle = deg2rad( double(angle) );
		
		tf.cTheta = cos(angle);
		tf.sTheta = sin(angle);

		std::cout << "Enter x \n";
		is >> tf.vector.x;

		std::cout << "Enter y \n";
		is >> tf.vector.y;
			
		return is;
	}
	
	/* Uses adjoint to map a twist from one frame to another
	 * See page 85 of Modern Robotics for more details. Since
	 * this is planar motion, the adjoint simplifies down to a pretty nice 
	 * little set of expressions. Initially, I constructed the whole 6x6 matrix
	 * but decided this was likely a better implementation
	 * twist_original - the input twist we are doing a change of frame on
	 * Return the twist in the new frame
	 */
	Twist2D Transform2D::operator()(Twist2D twist_original) {
			
		double w_new = twist_original.w;			
		
		double dx = (vector.y * twist_original.w) + (cTheta * twist_original.dx) - (sTheta * twist_original.dy);
		
		double dy = ((-1 * vector.x) * twist_original.w) + (sTheta * twist_original.dx) + (cTheta * twist_original.dy);

		return Twist2D(w_new, dx, dy);	
	}
	

	// End of Transform2D class 
	// Start of Twist2D class

	/* Constructor for Twist2D
	 */ 	
	Twist2D::Twist2D(double w, double dx, double dy) {
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/* Create the (0.0, 0.0, 0.0) twist
	 * One of a few diffrent constructors
	 */
	Twist2D::Twist2D(void) {
                this->w = 0.0;
                this->dx = 0.0;
                this->dy = 0.0;
        }
	
	/* Set/update a twist's fields 
	 */
	void Twist2D::setVars(double w, double dx, double dy) {
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/* Print a human readable description of the twist
	 * os - the output stream
	 * tw - the twist we are describing
	 */	
	std::ostream & operator<<(std::ostream & os, const Twist2D &tw) {
	
		std::cout << "\nAngular part: " << tw.w << "     dx: " << tw.dx << "    dy: " << tw.dy << std::endl;
		return os;
	}		

	/* Read in a Twist2d from the user
	 * is - the input stream
	 * tw - the twist object we are inputing
	 * Returns the input stream
	 */
	std::istream & operator>>(std::istream & is, Twist2D &tw) {
	
		std::cout << "Enter the angular part (rad/s) ";
		is >> tw.w;

		std::cout << "Enter dx ";
                is >> tw.dx;

		std::cout << "Enter dy ";
                is >> tw.dy;		

		return is;
	}
	
	/* Does comparsion of two Transform2Ds
	 * This is useful for testing to see if two transforms are equal.
	 * lhs - the left hand side Transform2D
	 * rhs - the right hand side Transform2D
	 */
	bool operator==(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {
		
		using namespace rigid2d;
	
		// FIX ME ? - let it be off by a percentage of the total size	
		double epsilon = 0.001;
		if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && (almost_equal(lhs.vector.y, rhs.vector.y, epsilon)) && 
				almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {
			return true;
		}

		return false;
	}

	 /* Does inequality check of two Transform2Ds
         * This is useful for testing to see if two transforms are equal
	 * lhs - the left hand side Transform2D
         * rhs - the right hand side Transform2D
         */
        bool operator!=(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {

                using namespace rigid2d;

		// FIX ME - let it be off by a percentage of the total size
		double epsilon = 0.001;
                if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && almost_equal(lhs.vector.y, rhs.vector.y, epsilon) && 
				almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {
        
			return false;
		}

			return true;
        }
	
	 /* Does comparsion of two Vector2Ds
         * This is useful for testing to see if two vectors are equal.
         * lhs - the left hand side Vector2D
         * rhs - the right hand side Vector2D
         */
        bool operator==(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
		double epsilon = 0.001;
		if ( (almost_equal(lhs.x, rhs.x, epsilon) ) && (almost_equal(lhs.y, rhs.y, epsilon) ) ) {
			return true;
		}	
		
		return false;
	}

        /* Does inequality check of two Vector2Ds
         * This is useful for testing to see if two vectors are equal.
         * lhs - the left hand side Vector2D
         * rhs - the right hand side Vector2D
         */
        bool operator!=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
		double epsilon = 0.01;
		if ( (almost_equal(lhs.x, rhs.x, epsilon)) && (almost_equal(lhs.y, rhs.y, epsilon) ) ) {
			return false;
		}

		return true;
	}
}
