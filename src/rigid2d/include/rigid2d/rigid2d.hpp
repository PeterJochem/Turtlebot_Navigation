#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>
//#include <Eigen/Dense>

namespace rigid2d {

	/// \brief PI.  Not in C++ standard until C++20.
	constexpr double PI = 3.14159265358979323846;

	/// \brief approximately compare two floating-point numbers using
	///        an absolute comparison
	/// \param d1 - a number to compare
	/// \param d2 - a second number to compare
	/// \param epsilon - absolute threshold required for equality
	/// \return true if abs(d1 - d2) < epsilon
	/// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
	/// be useful here
	constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12) {

		// WHY does fabs give weird issues?????
		if ( std::fabs(d2 - d1) < epsilon) {
			return true;
		}

		return false;
	}

	/// \brief convert degrees to radians
	/// \param deg - angle in degrees
	/// \returns radians
	/// NOTE: implement this in the header file
	/// constexpr means that the function can be computed at compile time
	/// if given a compile-time constant as input
	constexpr double deg2rad(double deg) {
		return (deg / 360.0) * (2 * PI);
	}

	/// \brief convert radians to degrees
	/// \param rad - angle in radians
	/// \returns the angle in degrees
	constexpr double rad2deg(double rad) {
		return (rad / (2 * PI) ) * 360.0;
	}

	/// static_assertions test compile time assumptions.
	/// You should write at least one more test for each function
	/// You should also purposely (and temporarily) make one of these tests fail
	/// just to see what happens
	static_assert(almost_equal(0, 0), "is_zero failed");
	static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");

	static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

	static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

	static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");


	/// \brief A 2-Dimensional Vector
	struct Vector2D {
		double x = 0.0;
		double y = 0.0;
	};

	/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
	/// os - stream to output to
	/// v - the vector to print
	std::ostream & operator<<(std::ostream & os, const Vector2D & v);


	class Twist2D {

		public:
			Twist2D(double, double, double);

			Twist2D(void);

			friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

			friend std::istream & operator>>(std::istream & is, const Twist2D & tw);

			void setVars(double, double, double);

			double getDx() const;
			double getDy() const;
			double getW() const;

			double w;
			double dx;
			double dy;

		private:
			/*
			   double w;
			   double dx;
			   double dy;
			   */

	};


	/// \brief a rigid body transformation in 2 dimensions
	class Transform2D {
		public:
			/// \brief Create an identity transformation
			Transform2D(void);

			/// \brief create a transformation that is a pure translation
			/// \param trans - the vector by which to translate
			explicit Transform2D(const Vector2D & trans);

			/// \brief create a pure rotation
			/// \param radians - angle of the rotation, in radians
			explicit Transform2D(double radians);

			/// \brief Create a transformation with a translational and rotational
			/// component
			/// \param trans - the translation
			/// \param rot - the rotation, in radians
			Transform2D(const Vector2D & trans, double radians);

			// \brief
			Transform2D(const Vector2D & trans, double cTheta, double sTheta);


			/// \brief apply a transformation to a Vector2D
			/// \param v - the vector to transform
			/// \return a vector in the new coordinate system
			Vector2D operator()(Vector2D v) const;

			/// \brief Given that this Transform2d represents T_ab
			// this operator will convert the twist in frame b into the 
			// equivalent twist represented in the A frame  
			// This method uses the adjoint mapping to convert the twist 
			/// \param twist - the twist defined in the B frame
			/// \return the twist represented in the A frame 
			Twist2D operator()(Twist2D twist);


			/// \brief invert the transformation
			/// \return the inverse transformation. 
			Transform2D inv() const;

			/// \brief compose this transform with another and store the result 
			/// in this object
			/// \param rhs - the first transform to apply
			/// \returns a reference to the newly transformed operator
			Transform2D & operator*=(const Transform2D & rhs);

			/// \brief \see operator<<(...) (declared outside this class)
			/// for a description
			friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

			double getX(void) const;
			double getY(void) const;
			double getTheta(void) const;
			double getCTheta(void) const;
			double getSTheta(void) const;

			Vector2D vector;
			double sTheta;
			double cTheta;

		private:
			//Vector2D vector;
			//double sTheta;
			//double cTheta;
	};


	/// \brief Tests equality of two Transform2D's. I set threshold for
	//  each field to be within about 0.001 of othe frames value
	//  \return true/false depending on if lhs is equal to rhs
	//
	bool operator==(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D &rhs);

	/// \brief For testing
	//
	bool operator!=(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D &rhs);

	/// \brief for testing
	bool operator==(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);

	/// \brief For testing
	//
	bool operator!=(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs);

	std::ostream & operator<<(std::ostream & os, const Twist2D &tw);

	std::istream & operator>>(std::istream & is, Twist2D &tw);	

	/// \brief should print a human readable version of the transform:
	/// An example output:
	/// dtheta (degrees): 90 dx: 3 dy: 5
	/// \param os - an output stream
	/// \param tf - the transform to print
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

	/// \brief Read a transformation from stdin
	/// Should be able to read input either as output by operator<< or
	/// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
	std::istream & operator>>(std::istream & is, Transform2D & tf);

	/// \brief multiply two transforms together, returning their composition
	/// \param lhs - the left hand operand
	/// \param rhs - the right hand operand
	/// \return the composition of the two transforms
	/// HINT: This function can be implemented in terms of *=
	Transform2D operator*(const Transform2D &lhs, const Transform2D &rhs);


}

#endif
