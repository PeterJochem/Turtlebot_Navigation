#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// @file
/// @brief Library for two-dimensional rigid body transformations.

#include<iosfwd> 
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace rigid2d {

	/// @brief PI.  Not in C++ standard until C++20.
	constexpr double PI = 3.14159265358979323846;

	/** @brief approximately compare two floating-point numbers using
	*          an absolute comparison
	* @param d1 - a number to compare
	* @param d2 - a second number to compare
	* @param epsilon - absolute threshold required for equality
	* @return true if abs(d1 - d2) < epsilon
	*/
	constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12) {

		if (std::fabs(d2 - d1) < epsilon) {
			return true;
		}

		return false;
	}
	

	/** @brief Maps angle into [-pi, pi] 
	* @param rad is the original angle in radians
	* @return the normalized angle
	*/
	constexpr double normalize_angle(double rad) {
		
		double p  = std::floor( (rad + PI) / (2.0 * PI) );
      		rad = (rad + PI) - p * 2.0 * PI;

      		if (rad < 0) {
        		rad = rad + 2.0 * PI;
      		}		
	
      		return rad - PI;
	}


	/** @brief convert degrees to radians
	* @param deg - angle in degrees
	* @return the angle in radians
	*/
	constexpr double deg2rad(double deg) {
		return (deg / 360.0) * (2 * PI);
	}

	/** @brief convert radians to degrees
	* @param rad - angle in radians
	* @return the angle in degrees
	*/
	constexpr double rad2deg(double rad) {
		return (rad / (2 * PI) ) * 360.0;
	}

	/** static_assertions test compile time assumptions.
	* You should write at least one more test for each function
	* You should also purposely (and temporarily) make one of these tests fail
	* just to see what happens
	*/
	static_assert(almost_equal(0, 0), "is_zero failed");
	static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
	static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
	static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
	static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

	/// @brief A 2-Dimensional Vector
	struct Vector2D {
		double x = 0.0;
		double y = 0.0;
	};
		
	/** @brief Normalizes a vector to be of unit length
	 *  @param orig_vector is the vector to be normalized
	 *  @return the normalized vector
	 */
	Vector2D normalize(Vector2D orig_vector);

	/** @brief output a 2 dimensional vector as [xcomponent ycomponent]
	*   @param os - stream to output to
	*   @param v - the vector to print
	*   @return the output stream */
	std::ostream & operator<<(std::ostream & os, const Vector2D & v);

	// Forward declare the class
	class Transform2D;

	class Twist2D {

		public:
			
			/** @brief Constructor for Twist2D
        		* @param w - the angular velocity
			* @param dx - the x velocity
			* @param dy - the y velocity */
			Twist2D(double w, double dx, double dy);
			
			/// @brief Default constructor. Creates the (0.0, 0.0, 0.0) twist
			Twist2D(void);
			
			/** @brief Print a human readable description of the twist
		         * @param os - the output stream
         		 * @param tw - the twist we are describing
			 * @return the output stream */
			friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);
			
			/** @brief Read in a Twist2d from the user
       			 * @param is - the input stream
       		 	 * @param tw - the twist object we are inputing
		         * @return the input stream */
			friend std::istream & operator>>(std::istream & is, const Twist2D & tw);
			
			/** @brief Set/update a twist's fields
         		* @params w - the angular velocity about the z-axis
         		* @params dx - cartesian x velocity in the plane
        		* @params dy - cartesian y velocity in the plane */
			void setVars(double, double, double);
			
			/** @brief Take a twist and scale it by dt
        		 * @return the scaled Twist */
			Twist2D scaleTwist(double);

			/** @brief Retrieves the x component of robot's position
		         * @return the x component of robot's position */
			double getDx() const;
			
			/** @brief Retrieves the y component of robot's position
		         * @return the x component of robot's position */
			double getDy() const;
			
			/** @brief Retrieves the angular component of robot's position
		         * @return the angle of the robot in the plane */
			double getW() const;
			
			double w;
			double dx;
			double dy;
	};


	/// \brief a rigid body transformation in 2 dimensions
	class Transform2D {
		public:
			/// @brief Default Constructor. Creates the identity transformation
			Transform2D(void);

			/** @brief Create a transformation that is only a transformation without any rotation
		        * @param p - the vector describing the frame's displacement */
			explicit Transform2D(const Vector2D & trans);

			/** Constructor for Transform2D which creates a twist which is just a rotation
		        * @param radians - the angle of rotation about the z-axis */
			explicit Transform2D(double radians);
			
			/** @brief Integrate a constant twist for unit time
			* So, if I am T_sb and I apply the given twist for 1s then
        		* I end up at T_bc. So, if I wanted T_sc, then I would need
 		        * to multiply T_sb * T_sc
       		 	* @param t - the twist
        		* @return the new frame relative to the old/prior one
		        * Important - this function assumes unit time! */
			Transform2D integrateTwist(Twist2D);
		
			/// \brief Create a transformation with a translational and rotational
			/// component
			/// \param trans - the translation
			/// \param rot - the rotation, in radians
			Transform2D(const Vector2D & trans, double radians);

			/** @brief Create a transformation which has both a rotation and a translation
	        	* @param trans - vector describing translation
        		* @param radians - orientation about z-axis */
			Transform2D(const Vector2D & trans, double cTheta, double sTheta);

			/** @brief Compute the x, y, theta of the Transform as a Pose
			* @return the x, y, theta of the Transform as a Pose */
			geometry_msgs::Pose displacement();
			
			/** @brief Apply the transformation to the vector v
        		* @param Vector2D - the vector we are applying the transformation to 
    		        * @return the resulting Vector2D */
			Vector2D operator()(Vector2D v) const;

			/** @brief Uses adjoint to map a twist from one frame to another
         		*  	   See page 85 of Modern Robotics for more details. Since
         		* 	   this is planar motion, the adjoint simplifies down to a pretty nice 
         		*          little set of expressions. Initially, I constructed the whole 6x6 matrix
         		*          but decided this was likely a better implementation
         		* @param twist_original - the input twist we are doing a change of frame on
         		* @return the twist in the new frame */
			Twist2D operator()(Twist2D twist);

				
			/** @brief Computes and returns the inverse of the given SE(2) matrix
         		* 	   Instead of constructing the entire matrix in Eigen and 
         		* 	   inverting, I opt to use the properties of SE(2) matrices.
         		*          This should help reduce the amount of numerical error 
         		*          See Modern Robotics chapter 3 for properties of SE(2) matrices  
         		* @return the normalized vector */
			Transform2D inv() const;
			
			/** @brief Does comparsion of two Transform2Ds
         		* 	   This is useful for testing to see if two transforms are equal.
         		* @param lhs - the left hand side Transform2D
         		* @param rhs - the right hand side Transform2D
       			* @return true if two SE(2) are equal (within small tolerance), false otherwise */
			Transform2D & operator*=(const Transform2D & rhs);

			/** @brief Print a human readable description of the twist
         		* @param os - the output stream
         		* @param tw - the twist we are describing 
			* @return the output stream */
			friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

			/** @brief Get and return x component of the twist
			 *  @return the x component of the twist */
			double getX(void) const;	
			
			/** @brief Get and return y component of the twist
			 *  @return the y component of the twist */
			double getY(void) const;
					
			/** @brief Uses the stored cos(theta) and sin(theta) fields to recover theta. 
        		*   @return theta in radians */
			double getTheta(void) const;
			
			/** @brief Retrieves the cos of the current angle
		         *  @return the cos of the current angle in the plane */
			double getCTheta(void) const;
			
			/** @brief Retreives the sin of the current angle
		         * @return the sin of the current angle in the plane */
			double getSTheta(void) const;
			
			Vector2D vector;
			double sTheta;
			double cTheta;
	};

	/** @brief Does comparsion of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal.
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return true if two SE(2) are equal (within small tolerance), false otherwise */
	bool operator==(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D &rhs);

	/** @brief Does inequality check of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return false if the two SE(2) are equal (within small tolerance), true otherwise */
	bool operator!=(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D &rhs);

	/** @brief Does comparsion of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal.
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return true if two SE(2) are equal (within small tolerance), false otherwise */
	bool operator==(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);

	/** @brief Does inequality check of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return false if the two SE(2) are equal (within small tolerance), true otherwise */
	bool operator!=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);
	
	/** @brief Print a human readable description of the twist
         * @param os - the output stream
         * @param tw - the twist we are describing 
	 * @return the output stream */	
	std::ostream & operator<<(std::ostream & os, const Twist2D &tw);

	/** @brief Read in a Twist2d from the user
         * @param is - the input stream
         * @param tw - the twist object we are inputing
         * @return the input stream */
	std::istream & operator>>(std::istream & is, Twist2D &tw);	
		
	/** @brief Print a human readable description of the Transform2D
        * @param os - the output stream
        * @param tf - the Transform2D we are describing
        * @return the output stream */
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
	
	/** @brief Reads in a Transform2D from the user
         * @param is - the input stream
         * @param tf - the Transform2D object we are updating/setting fields of
   	 * @return the input stream */
	std::istream & operator>>(std::istream & is, Transform2D & tf);
	
	/** @brief Compose two transformations together and return the result
         * @param lhs - the left hand side transformation
         * @param rhs - the right hand side transformation
         * Note: A 2D transformation matrix can be stored with only cTheta, sTheta,
         * x, and y. There's no need to multiply the entire matrix or store the entire matrix
         * @return the composition of the two transformations */
	Transform2D operator*(const Transform2D &lhs, const Transform2D &rhs);
	
	/** @brief Does comparsion of two Transform2Ds
         * 	   This is useful for testing to see if two transforms are equal.
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return true if two SE(2) are equal (within small tolerance), false otherwise */
        bool operator==(const rigid2d::Twist2D &lhs, const rigid2d::Twist2D &rhs);
	
	/** @brief Does inequality check of two Transform2Ds
         * @param lhs - the left hand side Transform2D
         * @param rhs - the right hand side Transform2D
         * @return false if the two SE(2) are equal (within small tolerance), true otherwise */
        bool operator!=(const rigid2d::Twist2D &lhs, const rigid2d::Twist2D &rhs);
	
	/** @brief Perform vector addition
         * @param lhs - the left hand side vector
         * @param rhs - the right hand side vector
         * @return the result of the vector addition */	
        Vector2D operator+(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);
			
	/** @brief Perform vector addition
          * @param lhs - the left hand side vector
          * @param rhs - the right hand side vector
          * @return the result of the vector addition */
	Vector2D operator+=(rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);

	/** @brief Perform vector subtraction
          * @param lhs - the left hand side vector 
          * @param rhs - right hand side vector
          * @return the result of the vector subtraction */
	Vector2D operator-(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);
		
	/** @brief Perform vector subtraction
         * @param lhs - the left hand side vector
         * @param rhs - the right hand side vector
         * @return the result of the vector subtraction */
	Vector2D operator-=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);	
	
	/** @brief Perform scalar multiplication of a vector
         * @param lhs - the left hand side vector
         * @param rhs - the scalar multiplier
         * @return the result of a scalar multiply of a vector */
	Vector2D operator*(const rigid2d::Vector2D &lhs, const double &rhs);
	
	/** @brief Perform scalar multiplication of a vector
         * @param rhs - the scalar multiplier
         * @param lhs - the vector
         * @return the result of a scalar multiply of the vector */
	Vector2D operator*(const double &rhs, const rigid2d::Vector2D &lhs);	
		
	/** @brief Perform scalar multiplication of a vector
         * @param lhs - the left hand side vector
         * @param rhs - the right hand side scalar multiplier 
         * @return the result */
        Vector2D operator*=(rigid2d::Vector2D &lhs, const double &rhs);
	
	/** @brief Compute the length of a vector
         * @param vector - the vector whose length we compute
         * @return the length of the vector */
        double length(const rigid2d::Vector2D &vector);
	
	/** @brief Compute and the angle of a vector
         *  @param vector - the vector whose angle we are computing
         *  @return the angle in radians */
	double angle(const rigid2d::Vector2D &vector);
	
	/** @brief Compute the distance between two vectors
         *  @param lhs - the left hand side vector
         *  @param rhs - the right hand side vector
         *  @return the distance between two vectors */
	double distance(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);	
		
	/** @brief Convert the geometry_msgs::Twist object to a rigid2d::Twist2D object
         *  @param twist - the twist in ROS format
         *  @return the twist in rigid2D format */
	Twist2D convert3DTo2D(geometry_msgs::Twist twist);	
}

#endif
