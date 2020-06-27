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
         *
        void Transform2D::setMatrices(double x, double y, double theta) {

                this->vector.x = x;
                this->vector.y = y;
                this->cTheta = cos(theta);
		this->sTheta = sin(theta);

                // Create the transformation matrix
                // Eigen uses column major order
                this->tf(0, 0) = cTheta;
                this->tf(0, 1) = -1 * sTheta;
                this->tf(0, 2) = x;

                this->tf(1, 0) = sTheta;
                this->tf(1, 1) = cTheta;
                this->tf(1, 2) = y;

                this->tf(2, 0) = 0.0;
                this->tf(2, 1) = 0.0;
                this->tf(2, 2) = 1.0;
        }*/

	/* Describe this method
         *
        void Transform2D::setMatrices(double x, double y, double cTheta, double sTheta) {

                this->vector.x = x;
                this->vector.y = y;
                this->cTheta = cTheta;
                this->sTheta = sTheta;

                // Create the transformation matrix
                // Eigen uses column major order
                this->tf(0, 0) = cTheta;
                this->tf(0, 1) = -1 * sTheta;
                this->tf(0, 2) = x;

                this->tf(1, 0) = sTheta;
                this->tf(1, 1) = cTheta;
                this->tf(1, 2) = y;

                this->tf(2, 0) = 0.0;
                this->tf(2, 1) = 0.0;
                this->tf(2, 2) = 1.0;
        }
	*/

	/* Create the identity transformation 
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


	/* Create a transformation that is just a transformation
	 */
	Transform2D::Transform2D(const Vector2D & trans) {
		
		cTheta = cos(0.0);
                sTheta = sin(0.0);
                vector.x = trans.x;
                vector.y = trans.y;
	}
	
	/* Create a Transform2D which is just a rotation
	 * Radians - the angle of rotation about the z-axis
	 */
	Transform2D::Transform2D(double radians) {
		
		cTheta = cos(radians);
                sTheta = sin(radians);
                vector.x = 0.0;
                vector.y = 0.0; 
	}
 
	/* Describe
	 */
	Transform2D::Transform2D(const Vector2D & trans, double cTheta, double sTheta) {
		
		this->cTheta = cTheta;
                this->sTheta = sTheta;
                this->vector.x = trans.x;
                this->vector.y = trans.y; 	
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
                
		// IS THIS RIGHT??
		// FIX ME FIX ME FIX ME FIX ME FIX ME
		// FIX ME FIX ME FIX ME
		// FIX ME FIX ME FIX ME
		// FIX ME FIX ME FIX ME
		// RANGE OF ACOS is [0, pi]
		return acos(cTheta);
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
	
	/*Eigen::Matrix<double, 3, 3> Transform2D::getTf(void) const {
                return tf;
        }
	*/
	
	/* The returns the inverse of the given SE(2) matrix
	 * Instead of constructing the entire matrix in Eigen and 
	 * inverting, I opt to use the properties of SE(2) matrices.
	 * This should help reduce the amount of numerical error  
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

	/* Describe this method
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
	 * Vector2D is the vector we are applying the transformation to 
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
	
	/* Describe this method 
	 * See page 85 of Modern Robotics
         *               [p] is the skew symmetric matrix of the three vector
         *               see page 65 of Modern Robotics
	 * I opted for doing the math symbolically and programming it in rather
	 * than creating the matrices in Eigen and multiplying. This is lighter weight
	 * and should be faster in the long run. Since we only rotate about the z axis,
	 * the expression for the adjoint is pretty simple, in comparison to the general
	 * purpose case
	 */
	Twist2D Transform2D::operator()(Twist2D twist_original) {
			
		double w_new = twist_original.w;			
		
		double dx = (vector.y * twist_original.w) + (cTheta * twist_original.dx) - (sTheta * twist_original.dy);
		
		double dy = ((-1 * vector.x) * twist_original.w) + (sTheta * twist_original.dx) + (cTheta * twist_original.dy);

		return Twist2D(w_new, dx, dy);	
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
	
	/* Describe 
	 * This is useful for testing to see if two transforms are equal
	 */
	bool operator==(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {
		
		using namespace rigid2d;
	
		// FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
		
		double epsilon = 0.001;
		if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && (almost_equal(lhs.vector.y, rhs.vector.y, epsilon)) && 
				almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {
			return true;
		}

		return false;
	}

	 /* Describe 
         * This is useful for testing to see if two transforms are equal
         */
        bool operator!=(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {

                using namespace rigid2d;

		// FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
		
		double epsilon = 0.001;
                if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && almost_equal(lhs.vector.y, rhs.vector.y, epsilon) && 
				almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {
        
			return false;
		}

			return true;
        }
	
	/* Describe
	 */
        bool operator==(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size
                // // FIX ME - let it be off by a percentage of the total size

		double epsilon = 0.001;
		if ( (almost_equal(lhs.x, rhs.x, epsilon) ) && (almost_equal(lhs.y, rhs.y, epsilon) ) ) {
			return true;
		}	
		
		return false;
	}

        /* Describe
        */
        bool operator!=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
		// // FIX ME - let it be off by a percentage of the total size
		// // FIX ME - let it be off by a percentage of the total size
		// // FIX ME - let it be off by a percentage of the total size
		
		double epsilon = 0.01;
		if ( (almost_equal(lhs.x, rhs.x, epsilon)) && (almost_equal(lhs.y, rhs.y, epsilon) ) ) {
			return false;
		}

		return true;
	}



		

}
