/** @file
* @brief rigid2d.cpp implements screw theory calculations useful for odometry and motion planning. */

#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h>

namespace rigid2d {
	
	/** @brief Print a vector2D in a human readable format
	*  @param os - The output stream
	*  @param v - the vector to be printed */
	std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
		return os << "[" << v.x << ", " << v.y <<  "]" << std::endl;  
	}

	/** @brief Default Constructor. Creates the identity transformation */ 	
	Transform2D::Transform2D (void) {
		
		cTheta = cos(0.0);
		sTheta = sin(0.0);
		vector.x = 0.0;
		vector.y = 0.0;
	}

	/** @brief Create a transformation which has both a rotation and a translation
	*   @param trans - vector describing translation
	*   @param radians - orientation about z-axis */ 
	Transform2D::Transform2D(const Vector2D & trans, double radians) {

		cTheta = cos(radians);
                sTheta = sin(radians);
                vector.x = trans.x;
                vector.y = trans.y;		
	}

	/** @brief Create a transformation that is only a transformation without any rotation
	*   @param p - Vector describing the frame's displacement */
	Transform2D::Transform2D(const Vector2D &p) {
		
		cTheta = cos(0.0);
                sTheta = sin(0.0);
                vector.x = p.x;
                vector.y = p.y;
	}

	/** @brief Constructor for Transform2D which creates a transform which is just a rotation
	*   @param radians - Angle of rotation about the z-axis */
	Transform2D::Transform2D(double radians) {
		
		cTheta = cos(radians);
                sTheta = sin(radians);
                vector.x = 0.0;
                vector.y = 0.0; 
	}
 
	/** @brief Constructor (one of a few) for the Transform2D class
	 *  @param p - The vector describing the frames translation
	 *  @param cTheta - cos of the Transform2D's angle about z axis
	 *  @param sTheta - sin of the Transform2D's angle about the z axis */
	Transform2D::Transform2D(const Vector2D & p, double cTheta, double sTheta) {
		
		this->cTheta = cTheta;
                this->sTheta = sTheta;
                this->vector.x = p.x;
                this->vector.y = p.y; 	
	}	

	
	/** @brief Print a human readable description of the Transform2D
	*   @param os - output stream
	*   @param tf - Transform2D we are describing 
	*   @return the output stream */
	std::ostream & operator<<(std::ostream &os, const Transform2D &tf) {
               return os << "\ndtheta (degrees): " << rad2deg(tf.getTheta()) << "    dx: " << tf.getX() 
		       << "    dy: " << tf.getY() << std::endl;
	}
	
	/* @brief Get and return x component of SE(2) matrix
	 *  @return x component of SE(2) matrix */	
	double Transform2D::getX(void) const {
		return vector.x;
	}
	
	/** @brief Get and return y component of SE(2)
	 *  @return y component of SE(2) */
	double Transform2D::getY(void) const {
                return vector.y;
        }
	
	/** @brief Uses the stored cos(theta) and sin(theta) fields to recover theta. 
	*   @return theta in radians */
	double Transform2D::getTheta(void) const {
                
		// Returns the principal value of the arc tangent of y/x, expressed in radians.
		// To compute the value, the function takes into account the sign of both 
		// arguments in order to determine the quadrant.
		// Note - the call is  double atan2(double y, double x);
		// Principal arc tangent of y/x, in the interval [-pi,+pi] radians
		return std::atan2(sTheta, cTheta);	
	}
	
	/** @brief Retrieves the cos of the current angle
	 *  @return the cos of the current angle in the plane */
	double Transform2D::getCTheta(void) const {
		return cTheta;
	}

	/** @brief Retreives the sin of the current angle
	 *  @return The sin of the current angle in the plane */ 
	double Transform2D::getSTheta(void) const {
                return sTheta;
        }

	/** @brief Retrieves the x component of robot's position
	 *  @return the x component of robot's position */ 
	double Twist2D::getDx(void) const {
                return dx;
        }
		
	/** @brief Retrieves the y component of robot's position
	 *  @return the x component of robot's position */
	double Twist2D::getDy(void) const {
                return dy;
        }
	
	/** @brief Retrieves the angular component of robot's position
         *  @return the angle of the robot in the plane */
	double Twist2D::getW(void) const {
                return w;
        }
	
	/** @brief Take a twist and scale it by dt
         *  @return the scaled Twist */
	Twist2D Twist2D::scaleTwist(double dt) {
		
		// Multiply each velocity by (dt/1) because this 
		// is a percentage of the total 1 s twist
		double newDx = dx * dt; 
		double newDy = dy * dt;
		double newDw = w * dt;
				
		return Twist2D(newDw, newDx, newDy);
	}

	/** @brief Normalizes a vector to be of unit length 
	 *  @return the normalized vector */
	Vector2D normalize(Vector2D orig_vector) {
	
		double magnitude = sqrt(pow(orig_vector.x, 2) + pow(orig_vector.y, 2));
	
		Vector2D new_vector;
		new_vector.x = (orig_vector.x)/magnitude;
		new_vector.y = (orig_vector.y)/magnitude;

		return new_vector;
	}

	/** @brief  Comptes and returns the inverse of the given SE(2) matrix
         * 	    Instead of constructing the entire matrix in Eigen and 
         * 	    inverting, I opt to use the properties of SE(2) matrices.
         * 	    This should help reduce the amount of numerical error 
         * 	    See Modern Robotics chapter 3 for properties of SE(2) matrices  
         * @return the normalized vector */	
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
	

	/** @brief Updates current Transform2D by multiplying it by another Transform2D
	*   @param rhs - Transform2D we are composing the current Transform2D with
	*   @return The updated Transform2D  */
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
	
	/** @brief Apply the transformation to the vector v
	*   @param Vector2D - the vector we are applying the transformation to 
	*   @returns the resulting Vector2D */
	Vector2D Transform2D::operator()(Vector2D v) const {

		Vector2D newVector;
		newVector.x = (cTheta * v.x) + (-1 * sTheta * v.y) + vector.x; 	
		newVector.y = (sTheta * v.x) + (cTheta * v.y) + vector.y;
		return newVector;
	}

	/** @brief Compose two transformations together and return the result
	 *  @param lhs - the left hand side transformation
	 *  @param rhs - the right hand side transformation
	 *  Note: A 2D transformation matrix can be stored with only cTheta, sTheta,
	 *  x, and y. There's no need to multiply the entire matrix or store the
	 *  entire matrix
	 * @return the composition of the two transformations */ 
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

	/*! @brief Integrate a constant twist for unit time
        * 	   Return the Transform2D of the new frame position
        * 	   and orientation. So, if I am T_sb and I apply the given twist for 1s then
        * 	   I end up at T_bc. So, if I wanted T_sc, then I would need
        *          to multiply T_sb * T_sc
	*  @param t - the twist
        *  @returns the new frame relative to the old/prior one
        *  Important - this function assumes unit time! */
        rigid2d::Transform2D Transform2D::integrateTwist(Twist2D t) {
                using namespace rigid2d;
                // Avoid modyfing the original twist? 
		
                Vector2D p;
                double cTheta;
                double sTheta;
                
		// Should I have a higher threshold??
                // if ( w < 0.01) {
                if (!almost_equal(t.w, 0.0)) {
                        // Normalize (w, dx, dy) so that the angular component is 1 
                        double theta = std::abs(t.w);
                        double w_norm = t.w / std::abs(t.w);
                        
			// magnitude should be positive!!
                        double dx_norm = t.dx / std::abs(t.w);
                        double dy_norm = t.dy / std::abs(t.w);
                        
			// Solve for cTheta and sTheta  
                        cTheta = 1 + ( (1 - cos(theta) ) * (-1) * (w_norm * w_norm) );
                        sTheta = sin(theta) * w_norm;
                        
			// Solve for the displacement
                        double A = ( theta + ( (theta - sin(theta) ) * (-1) * (w_norm * w_norm) ) );
                        double B = ( (1 - cos(theta) ) * (-1) * (w_norm) );
                        
			p.x = (A * dx_norm) + (B * dy_norm);
                        p.y = (-B * dx_norm) + (A * dy_norm);
                }
                else {
                        // Normalize (w, dx, dy) so that the v component is 1
                        double w_norm = 0.0;
                        double magnitude = sqrt( (t.dx * t.dx) + (t.dy * t.dy) );
                        
			double dx_norm = t.dx / magnitude;
                        double dy_norm = t.dy / magnitude;
                        
			cTheta = 1.0; // cos(w_norm);
                        sTheta = 0.0; // sin(w_norm);   
                        
			p.x = t.dx;
                        p.y = t.dy;
                }

                return (*this * Transform2D(p, cTheta, sTheta) );
        }


	/** @brief Reads in a Transform2D from the user
	 *  @param is - the input stream
       	 *  @param tf - the Transform2D object we are updating/setting fields of */	
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
	
	/** @brief Uses adjoint to map a twist from one frame to another
	 * 	   See page 85 of Modern Robotics for more details. Since
	 * 	   this is planar motion, the adjoint simplifies down to a pretty nice 
	 *  	   little set of expressions. Initially, I constructed the whole 6x6 matrix
	 * 	   but decided this was likely a better implementation
	 *  @param twist_original - the input twist we are doing a change of frame on
	 *  @return the twist in the new frame */
	Twist2D Transform2D::operator()(Twist2D twist_original) {
	
		double w_new = twist_original.w;			
		double dx = (vector.y * twist_original.w) + (cTheta * twist_original.dx) - (sTheta * twist_original.dy);
		double dy = ((-1 * vector.x) * twist_original.w) + (sTheta * twist_original.dx) + (cTheta * twist_original.dy);	
		return Twist2D(w_new, dx, dy);	
	}
	

	/** @brief Constructor for Twist2D 
	 *  @param w - angular part of twist
	 *  @param dx - the x component of the twist
	 *  @param dy - the y component of the twist */ 	
	Twist2D::Twist2D(double w, double dx, double dy) {
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/// @brief Default constructor. Creates the (0.0, 0.0, 0.0) twist
	Twist2D::Twist2D(void) {
                this->w = 0.0;
                this->dx = 0.0;
                this->dy = 0.0;
        }
	
	/** @brief Set/update a twist's fields
	 *  @param w - the angular velocity about the z-axis
	 *  @param dx - cartesian x velocity in the plane
	 *  @param dy - cartesian y velocity in the plane */ 
	void Twist2D::setVars(double w, double dx, double dy) {
		this->w = w;
		this->dx = dx;
		this->dy = dy;
	}

	/** @brief Print a human readable description of the twist
	 *  @param os - Output stream
	 *  @param tw - Twist we are describing */	
	std::ostream & operator<<(std::ostream & os, const Twist2D &tw) {
	
		std::cout << "\nAngular part: " << tw.w << "     dx: " << tw.dx << "    dy: " << tw.dy << std::endl;
		return os;
	}		

	/** @brief Read in a Twist2d from the user
	 *  @param is - Input stream
	 *  @param tw - Twist object we are inputing
	 *  @return the input stream */
	std::istream & operator>>(std::istream & is, Twist2D &tw) {
	
		std::cout << "Enter the angular part (rad/s) ";
		is >> tw.w;

		std::cout << "Enter dx ";
                is >> tw.dx;

		std::cout << "Enter dy ";
                is >> tw.dy;		

		return is;
	}
	
	/** @brief Does comparsion of two Transform2Ds
	 * 	   This is useful for testing to see if two transforms are equal.
	 *  @param lhs - Left hand side Transform2D
	 *  @param rhs - Right hand side Transform2D
	 *  @return true if two SE(2) are equal (within small tolerance), false otherwise */
	bool operator==(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {
		
		using namespace rigid2d;
	
		double epsilon = 0.1;
		if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && (almost_equal(lhs.vector.y, rhs.vector.y, epsilon)) && 
				almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {
			return true;
		}

		return false;
	}

	/** @brief Does inequality check of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal
	 * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
	 * @return false if the two SE(2) are equal (within small tolerance), true otherwise */
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
	
	/** @brief Does comparsion of two Vector2Ds
         * 	   This is useful for testing to see if two vectors are equal.
         *  @param lhs - the left hand side Vector2D
         *  @param rhs - the right hand side Vector2D
	 *  @return true if two vectors are equal (within small tolerance), false otherwise */
        bool operator==(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
		double epsilon = 0.001;
		if ((almost_equal(lhs.x, rhs.x, epsilon)) && (almost_equal(lhs.y, rhs.y, epsilon))) {
			return true;
		}	
		
		return false;
	}

        /** @brief Does inequality check of two Vector2Ds
         * 	   This is useful for testing to see if two vectors are equal.
         *  @param lhs - the left hand side Vector2D
         *  @param rhs - the right hand side Vector2D
	 *  @return false if two vectors are equal (within small tolerance), true otherwise */
        bool operator!=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

		// FIX ME - let it be off by a percentage of the total size
		double epsilon = 0.01;
		if ((almost_equal(lhs.x, rhs.x, epsilon)) && (almost_equal(lhs.y, rhs.y, epsilon))) {
			return false;
		}

		return true;
	}

	/** @brief Does comparsion of two Twist2Ds
         * 	   This is useful for testing to see if two twists are equal.
         * @param lhs - the left hand side Twist2D
         * @param rhs - the right hand side Twist2D
	 * @return true if two twists are equal (within small tolerance), false otherwise */
        bool operator==(const rigid2d::Twist2D &lhs, const rigid2d::Twist2D &rhs) {

                // FIX ME - let it be off by a percentage of the total size
                double epsilon = 0.1;
                if ( (almost_equal(lhs.dx, rhs.dx, epsilon) ) && (almost_equal(lhs.dy, rhs.dy, epsilon) )
		 	&& (almost_equal(lhs.w, rhs.w, epsilon) ) ) {
                        return true;
                }

                return false;
        }

        /** @brief Does inequality check of two Twist2Ds
         * 	   This is useful for testing to see if two twists are equal.
         *  @param lhs - the left hand side Twist2D
         *  @param rhs - the right hand side Twist2D
	 *  @return false if two twists are equal (within small tolerance), true otherwise */
        bool operator!=(const rigid2d::Twist2D &lhs, const rigid2d::Twist2D &rhs) {

                // FIX ME - let it be off by a percentage of the total size
                double epsilon = 0.1;
                if ( (almost_equal(lhs.dx, rhs.dx, epsilon)) && (almost_equal(lhs.dy, rhs.dy, epsilon) ) 
				&& (almost_equal(lhs.w, rhs.w, epsilon) ) ) {
                        return false;
                }

                return true;
        }

	/** @brief Perform vector addition
	 *  @param lhs - the left hand side vector
	 *  @param rhs - the right hand side vector
	 *  @\return the result of the vector addition */
	Vector2D operator+(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {
		
		Vector2D result;
		result.x = lhs.x + rhs.x;
		result.y = lhs.y + rhs.y;

		return result;
	}

         /** @brief Perform vector addition
	  *  @param lhs - the left hand side vector
	  *  @param rhs - the right hand side vector
          *  @return the result of the vector addition */
	Vector2D operator+=(rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

                lhs.x = lhs.x + rhs.x;
                lhs.y = lhs.y + rhs.y;

                return lhs;
        }

	 /** @brief Perform vector subtraction
	  *  @param lhs - the left hand side vector 
	  *  @param rhs - right hand side vector
          *  @return the result of the vector subtraction */
        Vector2D operator-(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

                Vector2D result;
                result.x = lhs.x - rhs.x;
                result.y = lhs.y - rhs.y;

                return result;
        }

	/** @brief Perform vector subtraction
	 *  @param lhs - the left hand side vector
	 *  @param rhs - the right hand side vector
         *  @return the result of the vector subtraction */
        Vector2D operator-=(rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {

                lhs.x = lhs.x - rhs.x;
                lhs.y = lhs.y - rhs.y;

                return lhs;
        }
	
	/** @brief Perform scalar multiplication of a vector 
	 *  @param lhs - the left hand side vector
	 *  @param rhs - the scalar multiplier
	 *  @return the result of a scalar multiply of a vector */
	Vector2D operator*(const rigid2d::Vector2D &lhs, const double &rhs) { 
		
		Vector2D result;
		result.x =  lhs.x * rhs;
		result.y = lhs.y * rhs;

		return result;
	}
        
	/** @brief Perform scalar multiplication of a vector 
         *  @param rhs - the scalar multiplier  
	 *  @param lhs - the vector
	 *  @return the result of a scalar multiply of the vector */
	Vector2D operator*(const double &rhs, const rigid2d::Vector2D &lhs) {

                Vector2D result;
                result.x =  lhs.x * rhs;
                result.y = lhs.y * rhs;

                return result;
        }
	
	/** @brief Perform scalar multiplication of a vector
	 *  @param lhs - The left hand side vector
	 *  @param rhs - The right hand side scalar multiplier 
	 *  @return The result */
        Vector2D operator*=(rigid2d::Vector2D &lhs, const double &rhs) {

                lhs.x = lhs.x * rhs;
                lhs.y = lhs.y * rhs;

                return lhs;
        }

	/** @brief Compute the length of a vector
	 *  @param vector - the vector whose length we compute
	 *  @return the length of the vector */
	double length(const rigid2d::Vector2D &vector) { 

		return sqrt( (vector.x * vector.x) + (vector.y * vector.y) ); 
	}

	/** @brief Compute and the angle of a vector
	 *  @param vector - The vector whose angle we are computing
	 *  @returns The angle in radians  */
        double angle(const rigid2d::Vector2D &vector) { 
                
		// atan2 takes into account the sign of both 
		// arguments in order to determine the quadrant.	
                return std::atan2(vector.y, vector.x);
        }

	/** @brief Compute the distance between two vectors
	 *  @param lhs - The left hand side vector
	 *  @param rhs - The right hand side vector
	 *  @return The distance between two vectors */
	double distance(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) { 
		
		return sqrt( std::pow(lhs.x - rhs.x, 2) + std::pow(lhs.y - rhs.y, 2) );	
	}
	
	/** @brief Convert the geometry_msgs::Twist object to a rigid2d::Twist2D object
	 *  @param twist - The twist in ROS format
	 *  @return The twist in rigid2D format */
	Twist2D convert3DTo2D(geometry_msgs::Twist twist) {
		
		double dx = twist.linear.x; 
		double dy = twist.linear.y;
		double dw = twist.angular.z;
		
		// Twist2D::Twist2D(double w, double dx, double dy)		
		return Twist2D(dw, dx, dy);
	}
}
