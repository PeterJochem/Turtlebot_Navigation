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
	
	/* Take the homogenous transformation matrix and return
        * the 3x3 rotation matrix from it
        */     
        Eigen::Matrix<float, 3, 3> Transform2D::extract_rotation() {
		
		Eigen::Matrix<float, 3, 3> R;

		R(0, 0) = tf(0, 0);
		R(0, 1) = tf(0, 1);
		R(0, 2) = tf(0, 2);
		
		R(1, 0) = tf(1, 0);
		R(1, 1) = tf(1, 1);
		R(1, 2) = tf(1, 2);
		
		R(2, 0) = tf(2, 0);
		R(2, 1) = tf(2, 1);
		R(2, 2) = tf(2, 2);

		return R;	
	}
	
	/* Take a 2x1 vector and return the 3x3 skew symmetric represntation
	 * of the vector, assuming its 3rd row were 0.0 
	 * Return a Eigen<float 3, 3> matrix which is the skew symmetric representation
	 */
	Eigen::Matrix<float, 3, 3> skew_sym(rigid2d::Vector2D vector) {
		
		Eigen::Matrix<float, 3, 3> M;
		
		M(0, 0) = 0.0; 
		M(0, 1) = 0.0;
		M(0, 2) = vector.y;

		M(1, 0) = 0.0;
		M(1, 1) = 0.0;
		M(1, 2) = -1 * vector.x;

		M(2, 0) = -1 * vector.y;
		M(2, 1) = vector.x;
		M(2, 2) = 0.0;

		return M;
	}	

	/* Take the two 3x3 matrices and construct the adjoint from them
	 */ 
	Eigen::Matrix<float, 6, 6> insertMatrices(Eigen::Matrix<float, 3, 3> R, Eigen::Matrix<float, 3, 3> lower_left_matrix) {

		Eigen::Matrix<float, 6, 6> M;

		// Insert R into the top left quadrant 
                M(0, 0) = R(0, 0);
                M(0, 1) = R(0, 1);
                M(0, 2) = R(0, 2);

		M(1, 0) = R(1, 0);
                M(1, 1) = R(1, 1);
                M(1, 2) = R(1, 2);

		M(2, 0) = R(2, 0);
                M(2, 1) = R(2, 1);
                M(2, 2) = R(2, 2);


		// Insert 0 into the top right quadrant 
		M(0, 3) = 0.0;
                M(0, 4) = 0.0;
                M(0, 5) = 0.0;

                M(1, 3) = 0.0;
                M(1, 4) = 0.0;
                M(1, 5) = 0.0;

                M(2, 3) = 0.0;
                M(2, 4) = 0.0;
                M(2, 5) = 0.0;


		// Insert [p]R into the lower left quadrant
		M(3, 0) = lower_left_matrix(0, 0);
                M(3, 1) = lower_left_matrix(0, 1);
                M(3, 2) = lower_left_matrix(0, 2);
			
                M(4, 0) = lower_left_matrix(1, 0);
                M(4, 1) = lower_left_matrix(1, 1);
                M(4, 2) = lower_left_matrix(1, 2);

                M(5, 0) = lower_left_matrix(2, 0);
                M(5, 1) = lower_left_matrix(2, 1);
                M(5, 2) = lower_left_matrix(2, 2);

   	
		// Insert R into the lower right quadrant
		M(3, 3) = R(0, 0);
                M(3, 4) = R(0, 1);
                M(3, 5) = R(0, 2);

                M(4, 3) = R(1, 0);
                M(4, 4) = R(1, 1);
                M(4, 5) = R(1, 2);

                M(5, 3) = R(2, 0);
                M(5, 4) = R(2, 1);
                M(5, 5) = R(2, 2);

                return M;
	}
	
	 /* Construct the adjoint of the given transformation matrix
         */
         Eigen::Matrix<float, 6, 6> Transform2D::adjoint() {
			
		Eigen::Matrix<float, 6, 6> adj;  
		
		Eigen::Matrix<float, 3, 3> R = extract_rotation();	
		
		// [p]R
		Eigen::Matrix<float, 3, 3> lower_left_matrix = skew_sym(vector) * R;
		
		// Construct the final 6x6 matrix
		adj = insertMatrices(R, lower_left_matrix);
			
		return adj;
        }


	/* Take the 2D twist and represent it as the general 6x1 vector
	 * Assuming we only rotate about the z-axis and can only translate
	 * in the x-y plane
	 */
	Eigen::Matrix<float, 6, 1> create_3D_Twist(rigid2d::Twist2D original_twist) {

		Eigen::Matrix<float, 6, 1> V;  
		V(0, 0) = 0.0;
		V(1, 0) = 0.0; 
		V(2, 0) = original_twist.getW();
		V(3, 0) = original_twist.getDx();
		V(4, 0) = original_twist.getDy();
		V(5, 0) = 0.0;
		
		return V;
	}

	/* Describe this method 
	 * See page 85 of Modern Robotics
         *               [p] is the skew shymmetric matrix of the three vector
         *               see page 65 of Modern Robotics
	 */
	Twist2D Transform2D::operator()(Twist2D twist_original) {
			
		// I defined the adjoint using the 3-d, general purpose approach
		// I will need to convert the 6, 1 vector back to the 1,3 vector for
		// the 2-d twists
		Eigen::Matrix<float, 6, 1> full_twist = adjoint() * create_3D_Twist(twist_original); 
			
		double w = full_twist(2, 0);
		double dx = full_twist(3, 0);
		double dy = full_twist(4, 0);

		// w (angular component) goes first??
		return rigid2d::Twist2D(w, dx, dy);
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

		if (almost_equal(lhs.getX(), rhs.getX()) && (almost_equal(lhs.getY(), rhs.getY())) && almost_equal(lhs.getTheta(), rhs.getTheta())) {
			return true;
		}

		return false;
	}

	 /* Describe 
         * This is useful for testing to see if two transforms are equal
         */
        bool operator!=(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {

                using namespace rigid2d;

                if (almost_equal(lhs.getX(), rhs.getX()) && (almost_equal(lhs.getY(), rhs.getY())) && almost_equal(lhs.getTheta(), rhs.getTheta())) {
                        return false;
                }

                return true;
        }


		

}
