#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <gtest/gtest.h>

/* Describe 
 * See test5.json
 */
TEST(rigid2D, Transform2D_Conversion) {

	using namespace rigid2d;

	// T_ab and T_bc are frames I choose
	Vector2D p;
	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ab = Transform2D(p, deg2rad(78.5) );


	p.y = 0.0;
	p.x = 0.0;
	Transform2D T_bc = Transform2D(p, deg2rad(-48.0) );

	// I computed these frames by hand - compositions/inv()
	// of T_ab and T_bc     

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ba_correct = Transform2D(p, 0.199368, -0.979925);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_cb_correct = Transform2D(p, 0.669131, 0.743145);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ca_correct = Transform2D(p, 0.861629, -0.507538);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ac_correct = Transform2D(p, 0.861629, 0.507538);


	// Compute the compositions of Transform2D's
	Transform2D T_ba = T_ab.inv();
	Transform2D T_ac = T_ab * T_bc;
	Transform2D T_ca = T_ac.inv();


	Twist2D twist_a_correct = Twist2D(1.0, -20.409, 18.2);

	Twist2D twist_b_correct = Twist2D(1.0, 13.7496, 23.7072);

	Twist2D twist_c_correct = Twist2D(1.0, -8.41758, 26.0811);

	// Compute the twist in the A and B frames      
	Twist2D twist_c = twist_c_correct;

	Twist2D twist_b = T_bc(twist_c);

	Twist2D twist_a = (T_ab * T_bc)(twist_c);

	assert( T_ba_correct == T_ba);
	assert( T_ca_correct == T_ca);
	assert( T_ac_correct == T_ac);

	assert( twist_a_correct == twist_a );
	assert( twist_b_correct == twist_b );
	assert( twist_c_correct == twist_c ); 	
}


/* Describe 
 * See test4.json
 * Add the twists too
 */
TEST(rigid2D, Transform2D_Conversion_2) {

	using namespace rigid2d;

	// T_ab and T_bc are frames I choose
	Vector2D p;
	p.x = 30000.000;
	p.y = -22.500;
	Transform2D T_ab = Transform2D(p, deg2rad(0.0) );

	p.y = 0.0;
	p.x = 0.0;
	Transform2D T_bc = Transform2D(p, deg2rad(12.1) );

	// I computed these frames by hand - compositions/inv()
	// of T_ab and T_bc	
	p.x = -30000.0000;
	p.y = 22.5000;
	Transform2D T_ba_correct = Transform2D(p, 1.0, 0.0);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_cb_correct = Transform2D(p, 0.977783, -0.2096185);

	p.x = -29328.800;
	p.y = 6310.5600;
	Transform2D T_ca_correct = Transform2D(p, 0.977783, -0.209619);

	p.x = 30000.0000;
	p.y = -22.5000;
	Transform2D T_ac_correct = Transform2D(p, 0.977783, 0.209619);


	// Compute the compositions of Transform2D's
	Transform2D T_ba = T_ab.inv();
	Transform2D T_ac = T_ab * T_bc;
	Transform2D T_ca = T_ac.inv();

	assert( T_ba_correct == T_ba);
	assert( T_ca_correct == T_ca);
	assert( T_ac_correct == T_ac);
}



/* Describe 
 * see test3.json
 */
TEST(rigid2D, Transform2D_Conversion_3) {

	using namespace rigid2d;

	// Angle should be in radians
	// T_ab and T_bc are frames I choose
	Vector2D p;
	p.x = 100.5;
	p.y = 0.45;
	Transform2D T_ab = Transform2D(p, deg2rad(0.0) );
	p.y = 0.0;   
	p.x = 0.0;
	Transform2D T_bc = Transform2D(p, deg2rad(30.0) );

	// I computed these frames by hand - compositions/inv()
	// of T_ab and T_bc
	p.x = -100.5; 
	p.y = -0.45;
	Transform2D T_ba_correct = Transform2D(p, 1.0, 0.0);

	p.x = 0.0;  
	p.y = 0.0;
	Transform2D T_cb_correct = Transform2D(p, 0.8660254037, -0.5);

	p.x = -87.2606;
	p.y = 49.8603;
	Transform2D T_ca_correct = Transform2D(p, 0.8660254037, -0.5);

	p.x = 100.5;
	p.y = 0.45;
	Transform2D T_ac_correct = Transform2D(p, 0.8660254037, 0.5);


	// Compute the compositions of Transform2D's
	Transform2D T_ba = T_ab.inv(); 	
	Transform2D T_ac = T_ab * T_bc; 
	Transform2D T_ca = T_ac.inv();


	assert( T_ba_correct == T_ba);		
	assert( T_ca_correct == T_ca);
	assert( T_ac_correct == T_ac);
}

/* Describe
 * see test2.json 
 */
TEST(rigid2D, Transform2D_Conversion_4) {

	using namespace rigid2d;

	// Angle should be in radians
	// T_ab and T_bc are frames I choose
	Vector2D p;
	p.x =  0.0;
	p.y = 0.0;
	Transform2D T_ab = Transform2D(p, deg2rad(0.0) );
	p.y = 0.0;
	p.x = 0.0;
	Transform2D T_bc = Transform2D(p, deg2rad(0.0) );

	// I computed these frames by hand - compositions/inv()
	// of T_ab and T_bc
	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ba_correct = Transform2D(p, 1.0, 0.0);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_cb_correct = Transform2D(p, 1.0, 0.0);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ca_correct = Transform2D(p, 1.0, 0.0);

	p.x = 0.0;
	p.y = 0.0;
	Transform2D T_ac_correct = Transform2D(p, 1.0, 0.0);

	// Compute the compositions of Transform2D's
	Transform2D T_ba = T_ab.inv();
	Transform2D T_ac = T_ab * T_bc;
	Transform2D T_ca = T_ac.inv();


	assert( T_ba_correct == T_ba);
	assert( T_ca_correct == T_ca);
	assert( T_ac_correct == T_ac);
}



/* Tests the inputting of Transform2Ds and twist conversions
 * see test1.json
 */
TEST(rigid2D, Transform2D_Conversion_5) {	

	using namespace rigid2d;

	// Use radians
	Vector2D v1;
	v1.x = 100.5;
	v1.y = 0.45;
	Transform2D T_ab = Transform2D(v1, deg2rad(180.0) );
	Vector2D v2;
	v2.x = 5.6;
	v2.y = -41.3;	
	Transform2D T_bc = Transform2D(v2, deg2rad(57.2958) );

	// Use rigid2d library to convert this to the a and b frames
	Twist2D twist_c = Twist2D(1.0, -4.70, 89.1);

	Twist2D twist_b = T_bc(twist_c);
	Twist2D twist_a = (T_ab * T_bc)(twist_c);


	// Make sure inputs and labels match    
	assert( almost_equal(T_ab.getX(), 100.5, 0.01) && (almost_equal(T_ab.getCTheta(), cos(deg2rad(180.0) ), 0.01) ) &&
			(almost_equal(T_ab.getSTheta(), sin(deg2rad(180.0)) , 0.01) ) ); 


	assert( (almost_equal(T_bc.getX(), 5.6, 0.01) && ( almost_equal(T_bc.getCTheta(), cos(deg2rad(57.2958) ) ), 0.01) ) && 
			(almost_equal(T_bc.getSTheta(), sin(deg2rad(57.2958) ) , 0.01) ) );

	// Test if converting a twist from one frame to another is correct
	assert( almost_equal(twist_b.w, 1.0, 0.01) && (almost_equal(twist_b.dx, -118.81450, 0.01) ) &&
			(almost_equal(twist_b.dy, 38.58599, 0.01) ) );

	assert( almost_equal(twist_a.w, 1.0, 0.01) && (almost_equal(twist_a.dx, 119.2644855, 0.01) ) &&
			(almost_equal(twist_a.dy, -139.0859945, 0.01) ) );			
}


/* Tests the constructor which creates the identity
 * transform  
 */
TEST(rigid2D, Transform2D_Identity_Constructor) {

	using namespace rigid2d;
	Transform2D tf = Transform2D();

	ASSERT_FLOAT_EQ(tf.vector.x, 0.0);
	ASSERT_FLOAT_EQ(tf.vector.y, 0.0);

	// Check that the angle is zero
	ASSERT_FLOAT_EQ(tf.getTheta(), 0.0);
	ASSERT_FLOAT_EQ(tf.sTheta, 0.0);
	ASSERT_FLOAT_EQ(tf.cTheta, 1.0);	
}

/* Tests the Transform2D constructor which 
 * creates the a transform with both a translation
 * and a rotation  
 */
TEST(rigid2D, Transform2D_Constructor_Translate_and_Rotate) {

	using namespace rigid2d;

	Vector2D p;
	p.x = 1234.567;
	p.y = -0.3241;
	double radians = -3.14/98.1;
	Transform2D tf = Transform2D(p, radians);

	ASSERT_FLOAT_EQ(tf.vector.x, p.x);
	ASSERT_FLOAT_EQ(tf.vector.y, p.y);
	ASSERT_FLOAT_EQ(tf.getTheta(), radians);
	ASSERT_FLOAT_EQ(tf.sTheta, sin(radians));
	ASSERT_FLOAT_EQ(tf.cTheta, cos(radians));
}

/* Tests the Transform2D constructor which
 * creates the a transform with both a translation
 * but no rotation
 */
TEST(rigid2D, Transform2D_Constructor_Translate) {

	using namespace rigid2d;

	Vector2D p;
	p.x = 1234.567;
	p.y = -0.3241;
	Transform2D tf = Transform2D(p);

	ASSERT_FLOAT_EQ(tf.vector.x, p.x);
	ASSERT_FLOAT_EQ(tf.vector.y, p.y);
	ASSERT_FLOAT_EQ(tf.getTheta(), 0.0);
	ASSERT_FLOAT_EQ(tf.sTheta, 0.0);
	ASSERT_FLOAT_EQ(tf.cTheta, 1.0);
}


/* Tests the Transform2D constructor which
 * creates the a transform with both a rotation
 * but no translation
 */
TEST(rigid2D, Transform2D_Rotate) {

	using namespace rigid2d;

	double radians = -3.14/1.32;
	Transform2D tf = Transform2D(radians);

	ASSERT_FLOAT_EQ(tf.vector.x, 0.0);
	ASSERT_FLOAT_EQ(tf.vector.y, 0.0);
	ASSERT_FLOAT_EQ(tf.getTheta(), radians);
	ASSERT_FLOAT_EQ(tf.sTheta, sin(radians));
	ASSERT_FLOAT_EQ(tf.cTheta, cos(radians));
}

/* Tests the Transform2D constructor which
 * creates the a transform with both a rotation
 * but translation but entered with vector, cTheta, sTheta
 * Also test the .getX(), .getY(), .getCTheta, .getSTheta methods
 */
TEST(rigid2D, Transform2D_Rotate_and_Translate) {

	using namespace rigid2d;

	Vector2D p;
	p.x = -2.345;
	p.y = -98123.89;
	double radians = -3.14/1.32;
	Transform2D tf = Transform2D(p, cos(radians), sin(radians));

	ASSERT_FLOAT_EQ(tf.getX(), p.x);
	ASSERT_FLOAT_EQ(tf.getY(), p.y);
	ASSERT_FLOAT_EQ(tf.getTheta(), radians);
	ASSERT_FLOAT_EQ(tf.getSTheta(), sin(radians));
	ASSERT_FLOAT_EQ(tf.getCTheta(), cos(radians));
}

/* Tests the vector normalization
*/
TEST(rigid2D, Vector2D_Normalization) {

	using namespace rigid2d;

	// Try normalizing a vector with magnitude > 1
	Vector2D p;
	p.x = -51.0; 
	p.y = 10.01;

	p = normalize(p);

	ASSERT_FLOAT_EQ(-0.98127740203, p.x);
	ASSERT_FLOAT_EQ(0.19259974106, p.y);

	// Try normalizing a vector with magnitude < 1
	p.x = -0.01;
	p.y = -0.32;

	p = normalize(p);

	ASSERT_FLOAT_EQ( -0.03123475237, p.x);
	ASSERT_FLOAT_EQ( -0.99951207609, p.y);
}

/* Test the inverse function
*/
TEST(rigid2D, Transform2D_Inverse) {

	using namespace rigid2d;

	Vector2D p;
	p.x = -1.2;
	p.y = 4.2;
	double radians = -3.14/3.0; 
	Transform2D tf_ab = Transform2D(p, radians);

	Transform2D tf_ba = tf_ab.inv();

	ASSERT_FLOAT_EQ(tf_ba.getX(), 4.236743);
	ASSERT_FLOAT_EQ(tf_ba.getY(), -1.0630189);
	ASSERT_FLOAT_EQ(tf_ba.getSTheta(), 0.86576);
	ASSERT_FLOAT_EQ(tf_ba.getCTheta(), 0.50045967);	


	// Test the inverse using a larger angle		
	radians = 14.1;
	tf_ab = Transform2D(p, radians);	

	tf_ba = tf_ab.inv();

	ASSERT_FLOAT_EQ(tf_ba.getX(), -4.1525092);
	ASSERT_FLOAT_EQ(tf_ba.getY(), -1.3552365);

	ASSERT_FLOAT_EQ(tf_ba.getSTheta(), -0.99930936);
	ASSERT_FLOAT_EQ(tf_ba.getCTheta(), 0.037158385);
}


/* Test the Transform2D's *= operator 
*/
TEST(rigid2D, Transform2D_Star_Equal) {

	using namespace rigid2d;

	Vector2D p;
	p.x = 30000.000;
	p.y = -22.500;
	Transform2D T_ab = Transform2D(p, deg2rad(0.0) );

	p.y = 0.0;
	p.x = 0.0;
	Transform2D T_bc = Transform2D(p, deg2rad(12.1) );

	// Actually T_ac =
	T_ab*=T_bc;

	ASSERT_FLOAT_EQ(T_ab.getX(), 30000.0000);
	ASSERT_FLOAT_EQ(T_ab.getY(), -22.5000);

	ASSERT_FLOAT_EQ(T_ab.getSTheta(), 0.20961857);
	ASSERT_FLOAT_EQ(T_ab.getCTheta(), 0.977783);
}


/* Test one of the Twist2D's constructors
 * It also tests the Twist2D's get methods
 */
TEST(rigid2D, Twist2D_Regular_Constructor) {

	using namespace rigid2d;

	// Twist2D::Twist2D(double w, double dx, double dy) {
	Twist2D t1 = Twist2D(5.1, 12.4, -0.98);

	ASSERT_FLOAT_EQ(t1.w, 5.1);
	ASSERT_FLOAT_EQ(t1.dx, 12.4);
	ASSERT_FLOAT_EQ(t1.dy, -0.98);

	ASSERT_FLOAT_EQ(t1.getW(), 5.1);
	ASSERT_FLOAT_EQ(t1.getDx(), 12.4);
	ASSERT_FLOAT_EQ(t1.getDy(), -0.98);	
}

/* Test the Twist2D's constructor for creating 
 * the (0.0, 0.0, 0.0) twist 
 * I also test updating a twist using setVars()
 */
TEST(rigid2D, Twist2D_Empty_Constructor) {

	using namespace rigid2d;

	// Twist2D::Twist2D(double w, double dx, double dy) {
	Twist2D t1 = Twist2D();

	ASSERT_FLOAT_EQ(t1.w, 0.0);
	ASSERT_FLOAT_EQ(t1.dx, 0.0);
	ASSERT_FLOAT_EQ(t1.dy, 0.0);

	t1.setVars(20.1, 0.5, -0.1);		
	ASSERT_FLOAT_EQ(t1.w, 20.1);
	ASSERT_FLOAT_EQ(t1.dx, 0.5);
	ASSERT_FLOAT_EQ(t1.dy, -0.1);
}

/* Test the Transform2D's comparison operators
*/
TEST(rigid2D, Transform2D_Comparison_Ops) {

	using namespace rigid2d;

	Vector2D p;
        p.x = 30000.000;
        p.y = -22.500;
        Transform2D T_ab = Transform2D(p, deg2rad(0.0) );

        p.y = 0.0;
        p.x = 0.0;
        Transform2D T_bc = Transform2D(p, deg2rad(12.1) );
	
	ASSERT_TRUE(T_ab == T_ab);
	ASSERT_TRUE(T_ab != T_bc);
	ASSERT_TRUE(T_bc == T_bc);
	ASSERT_TRUE(T_bc != T_ab);
}

/* Test the comparison functions for Vector2D
*/
TEST(rigid2D, Vector2D_Comparison_Ops) {

        using namespace rigid2d;

	Vector2D p;
	Vector2D q;
	Vector2D m;

	p.x = 0.1;
	p.y = 8.3;
	
	m.x = 0.1;
	m.y = 8.3;

	q.x = 103.5;
	q.y = -99.15;

	ASSERT_TRUE(p == p);
        ASSERT_TRUE(p != q);

	ASSERT_TRUE(p == p);
        ASSERT_TRUE(p != q);
	
	ASSERT_TRUE(p == m);
        ASSERT_TRUE(m == p);

	ASSERT_TRUE(q != m);
        ASSERT_TRUE(m != q);
}

/* Test the comparison functions for Twist2D
*/
TEST(rigid2D, Twist2D_Comparison_Ops) {

        using namespace rigid2d;

	Twist2D t1 = Twist2D(-95.1, 0.1, 200.0);
	Twist2D t2 = Twist2D(-95.1, 0.1, 200.0);
	Twist2D t3 = Twist2D(12.5, -87.5, -234.5);
	
	ASSERT_TRUE(t1 == t2);
	ASSERT_TRUE(t2 == t1);
	ASSERT_TRUE(t1 == t1);
	ASSERT_TRUE(t2 == t2);
        ASSERT_TRUE(t1 != t3);
        ASSERT_TRUE(t3 != t1);
        ASSERT_TRUE(t2 != t3);
}

/* Test the integrate twist function on a twist
 * with 0 angular component  
*/
TEST(rigid2D, IntegrateTwist) {

        using namespace rigid2d;

	// A linear translation in the +X direction
        Transform2D T_sb = Transform2D();  
	Twist2D t = Twist2D(0.0, 1.0, 0.0);	
	
	Transform2D T_sb_new = T_sb.integrateTwist(t);
	
	ASSERT_FLOAT_EQ(T_sb_new.getX(), 1.0);	
	ASSERT_FLOAT_EQ(T_sb_new.getTheta(), 0.0);
	
	
	t = Twist2D(0.0, 1.0, 100.0);
        T_sb_new = T_sb.integrateTwist(t);
			
	ASSERT_FLOAT_EQ(T_sb_new.getX(), 1.0);
	ASSERT_FLOAT_EQ(T_sb_new.getY(), 100.0);

	
	// Test one with only an angular component 
	t = Twist2D(3.14/2.0, 0.0, 0.0);
        T_sb_new = T_sb.integrateTwist(t);
			
	ASSERT_FLOAT_EQ(T_sb_new.getTheta(), 3.14/2.0);
	ASSERT_FLOAT_EQ(T_sb_new.getSTheta(), sin(3.14/2.0) );

	ASSERT_FLOAT_EQ(T_sb_new.getX(), 0.0);
        ASSERT_FLOAT_EQ(T_sb_new.getY(), 0.0);

	
	// Have the (0.0, 0.0, 0.0) twist
	t = Twist2D();
	T_sb_new = T_sb.integrateTwist(t);	
	ASSERT_FLOAT_EQ(T_sb_new.getTheta(), 0.0);
			
	// Use an example from Modern Robotics page 56
	t = Twist2D(3.14/2.0, 3.14, 0.0);		
	T_sb_new = T_sb.integrateTwist(t);
	
	ASSERT_FLOAT_EQ(T_sb_new.getTheta(), 3.14/2.0);
	ASSERT_FLOAT_EQ(T_sb_new.getSTheta(), sin(3.14/2.0) );
	ASSERT_FLOAT_EQ(T_sb_new.getCTheta(), cos(3.14/2.0) );
	
	ASSERT_TRUE( almost_equal( T_sb_new.getX(), 2.0, 0.01) );
	ASSERT_TRUE( almost_equal( T_sb_new.getY(), 2.0, 0.01) );
	
	// Non-zero component in each part of the twist
	t = Twist2D(1.0, 1.0, 1.0);
	
	Vector2D p;
	p.x = -1.0;
	p.y = 3.0;
	T_sb = Transform2D(p, rigid2d::PI/2);
	
	T_sb_new = T_sb.integrateTwist(t);
		
	ASSERT_TRUE( almost_equal(rad2deg(T_sb_new.getTheta()), 147.296, 0.001) );
		
	ASSERT_TRUE( almost_equal(T_sb_new.getX(), -2.30117, 0.001) );
	ASSERT_TRUE( almost_equal( T_sb_new.getY(), 3.38177, 0.001) );
}


/* Describe 
 */
TEST(rigid2D, Vector_Addition) {

	using namespace rigid2d;	

	Vector2D p;
	Vector2D q;
	Vector2D result;

	p.x = 1245.5;
	p.y = -89.4;

	q.x = 2.0;
	q.y = -0.1;

	result = p + q;

	ASSERT_FLOAT_EQ(result.x, 1247.5);
	ASSERT_FLOAT_EQ(result.y, -89.5);
	
	ASSERT_FLOAT_EQ(p.x, 1245.5);
        ASSERT_FLOAT_EQ(p.y, -89.4);

	ASSERT_FLOAT_EQ(q.x, 2.0);
        ASSERT_FLOAT_EQ(q.y, -0.1);

	result+=q;

	ASSERT_FLOAT_EQ(result.x, 1249.5);
        ASSERT_FLOAT_EQ(result.y, -89.6);

        ASSERT_FLOAT_EQ(p.x, 1245.5);
        ASSERT_FLOAT_EQ(p.y, -89.4);

        ASSERT_FLOAT_EQ(q.x, 2.0);
        ASSERT_FLOAT_EQ(q.y, -0.1);

}


TEST(rigid2D, Vector_Multiplication) {

	using namespace rigid2d;
	double x = 12.41;			
	
	Vector2D p;
	Vector2D result;
	p.x = 1.2;
	p.y = -9.3;
	
	// Multiply from the left
	result = p * x;
	
	ASSERT_FLOAT_EQ(result.x, 14.892);
        ASSERT_FLOAT_EQ(result.y, -115.413);	

	// Multiply from the right
	p.x = 1.2;
	p.y = -9.3;
	
	result = x * p;

	ASSERT_FLOAT_EQ(result.x, 14.892);
        ASSERT_FLOAT_EQ(result.y, -115.413);


	// Multiply by an integer
	p.x = 1.2;
        p.y = -9.3;
	
	double aScalar = 2;
	result = aScalar * p;
	
	ASSERT_FLOAT_EQ(result.x, 2.4);
        ASSERT_FLOAT_EQ(result.y, -18.6);

	// Multiply by an integer from the right
	p.x = 1.2;
        p.y = -9.3;

        result = p * aScalar;

        ASSERT_FLOAT_EQ(result.x, 2.4);
        ASSERT_FLOAT_EQ(result.y, -18.6);	
}

TEST(rigid2D, Vector_Star_Equals) {

        using namespace rigid2d;
        double x = 12.41;

        Vector2D p;
        Vector2D result;
        p.x = 1.2;
        p.y = -9.3;
		
	// Multiply by a double
	p*=2.0;
	
	ASSERT_FLOAT_EQ(p.x, 2.4);
        ASSERT_FLOAT_EQ(p.y, -18.6);
	
	// Multiply by a scalar
	p.x = 1.2;
	p.y = -9.3;
	
	int aScalar = 2;
	p*=aScalar;

	ASSERT_FLOAT_EQ(p.x, 2.4);
        ASSERT_FLOAT_EQ(p.y, -18.6);

	
	double aFloat = 87.4;
	p.x = 1.2;
	p.y = -9.3;
	p*=aFloat;

	ASSERT_FLOAT_EQ(p.x, 104.88);
        ASSERT_FLOAT_EQ(p.y, -812.82);
}

TEST(rigid2D, Vector2D_Length) {

        using namespace rigid2d;

        Vector2D p;
        p.x = 1.2;
        p.y = -9.3;

	ASSERT_FLOAT_EQ(length(p), 9.37709976485);

	p.x = 0.0;
	p.y = 0.0;

	ASSERT_FLOAT_EQ(length(p), 0.0);
}

TEST(rigid2D, Vector2D_Angle) {

        using namespace rigid2d;

        Vector2D p;
        p.x = 1.2;
        p.y = -9.3;

        ASSERT_TRUE( almost_equal( angle(p), -1.4425, 0.001) );

	p.x = 0.0;
        p.y = 0.0;

        ASSERT_TRUE( almost_equal(angle(p), 0.0, 0.0001) );
}

TEST(rigid2D, Vector2D_Distance) {

        using namespace rigid2d;

        Vector2D p;
        Vector2D q;
        p.x = 1.2;
        p.y = -9.3;
	
	q.x = -93.5;  
	q.y = 32.345;

        ASSERT_TRUE( almost_equal( distance(p, q), 103.452385, 0.001) );
	
        p.x = 1.0;
        p.y = 12.0;
	
	q.x = 1.0;
	q.y = 12.0;

        ASSERT_TRUE( almost_equal( distance(p, q), 0.0, 0.0001) );
}

TEST(rigid2D, Angle_Normalization) {

        using namespace rigid2d;

	double radians = 3 * PI;

	
	ASSERT_FLOAT_EQ( normalize_angle(radians), -PI);

	radians = 0.0;
	ASSERT_FLOAT_EQ( normalize_angle(radians), 0.0);

	radians = 0.12345;
	ASSERT_FLOAT_EQ( normalize_angle(radians), 0.12345);
	
	radians = -0.12345;
	ASSERT_FLOAT_EQ( normalize_angle(radians), -0.12345);
	

	radians = -1 * (PI * 1.5);
	ASSERT_FLOAT_EQ( normalize_angle(radians), PI/2.0);

	radians = -1 * (PI * 3.5);
        ASSERT_FLOAT_EQ( normalize_angle(radians), PI/2.0);

	radians = -1 * (PI * 3.5);
        ASSERT_FLOAT_EQ( normalize_angle(radians), PI/2.0);	
}

TEST(rigid2D, Diff_Drive_Constructor_1) {

        using namespace rigid2d;

	DiffDrive myDiff_Drive = DiffDrive();		
		
        ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getX(), 0.0);
	ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getY(), 0.0);
	ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getTheta(), 0.0);
}

TEST(rigid2D, Diff_Drive_Constructor_2) {

        using namespace rigid2d;
	
	Vector2D p;
	p.x = 98.45;
	p.y = -23.54;
	Transform2D T1 = Transform2D(p, 0.2345);
        DiffDrive myDiff_Drive = DiffDrive(T1, 2.0, 3.0);

        ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getX(), 98.45);
        ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getY(), -23.54);
        ASSERT_FLOAT_EQ( myDiff_Drive.current_pose.getTheta(), 0.2345);
}


TEST(rigid2D, Twist_To_Wheels) {

        using namespace rigid2d;
	
	Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
	DiffDrive myDiff_Drive = DiffDrive(T1, 5.0, 2.0);

	// Twist only has a translation in the x-axis
	Twist2D t = Twist2D(0.0, 1.0, 0.0);
	WheelVelocities wheel_vels = myDiff_Drive.twistToWheels(t);		
	
	ASSERT_FLOAT_EQ( wheel_vels.left, (1/myDiff_Drive.wheel_radius) );
	ASSERT_FLOAT_EQ( wheel_vels.right, (1/myDiff_Drive.wheel_radius) );

	// Another twist with only a translation in the x-direction
	t = Twist2D(0.0, -20.0, 0.0);
        wheel_vels = myDiff_Drive.twistToWheels(t); 

        ASSERT_FLOAT_EQ( wheel_vels.left, (-20.0/myDiff_Drive.wheel_radius) );
        ASSERT_FLOAT_EQ( wheel_vels.right, (-20.0/myDiff_Drive.wheel_radius) );

	// Twist only has a translation in the y-axis
	// Is this actually possible??
	// This violates the no slide constraint!
					
	rigid2d::DiffDrive bot;
  	rigid2d::Twist2D tw(1, 1, 0);
 	rigid2d::WheelVelocities vel;

  	vel = bot.twistToWheels(tw);

 	ASSERT_EQ(vel.right, 12.5);
  	ASSERT_EQ(vel.left, 7.5);
}

TEST(rigid2D, Wheels_To_Twist_Only_Translate) {
	
        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 5.0, 2.0);
	
	WheelVelocities vel;
	vel.left = 1.0;
	vel.right = 1.0;
	Twist2D Twist_robot = myRobot.wheelsToTwist(vel);
	
	ASSERT_FLOAT_EQ(Twist_robot.w, 0.0);
	ASSERT_FLOAT_EQ(Twist_robot.dx, 2.0);	
	ASSERT_FLOAT_EQ(Twist_robot.dy, 0.0);
		
        vel.left = -4.1;
        vel.right = -4.1;
        Twist_robot = myRobot.wheelsToTwist(vel);
		
	ASSERT_FLOAT_EQ(Twist_robot.w, 0.0);
        ASSERT_FLOAT_EQ(Twist_robot.dx, -8.2);
        ASSERT_FLOAT_EQ(Twist_robot.dy, 0.0);
}

TEST(rigid2D, Wheels_To_Twist_Rotate_Only) {

        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 5.0, 2.0);

        WheelVelocities vel;
        vel.left = 1.0;
        vel.right = -1.0;
        Twist2D Twist_robot = myRobot.wheelsToTwist(vel);

        ASSERT_FLOAT_EQ(Twist_robot.w, -4.0/5);
        ASSERT_FLOAT_EQ(Twist_robot.dx, 0.0);
        ASSERT_FLOAT_EQ(Twist_robot.dy, 0.0);
}


TEST(rigid2D, Wheels_To_Twist_Rotate_And_Translate) {

        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 0.5, 0.1);

        WheelVelocities vel;
        vel.left = 7.5;
        vel.right = 12.5;
        Twist2D Twist_robot = myRobot.wheelsToTwist(vel);

        ASSERT_FLOAT_EQ(Twist_robot.w, 1.0);
        //ASSERT_FLOAT_EQ(Twist_robot.dx, 0.25 + 1.0);
        ASSERT_FLOAT_EQ(Twist_robot.dx, 1.0);
	ASSERT_FLOAT_EQ(Twist_robot.dy, 0.0);
}


TEST(rigid2D, DiffDrive_Constructor_and_Pose) {
        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 0.5, 0.1);

	geometry_msgs::Pose2D pose = myRobot.pose();
        ASSERT_FLOAT_EQ( pose.x, 0.0);
        ASSERT_FLOAT_EQ( pose.y, 0.0);
        ASSERT_FLOAT_EQ( pose.theta, 0.0);

	p.x = 8.1;
        p.y = -20.0;

	T1 = Transform2D(p, 0.0);
	myRobot = DiffDrive(T1, 0.5, 0.1);
	
	pose = myRobot.pose();
        ASSERT_FLOAT_EQ( pose.x, 8.1);
        ASSERT_FLOAT_EQ( pose.y, -20.0);
        ASSERT_FLOAT_EQ( pose.theta, 0.0);
		
	p.x = -25.1;
        p.y = 99.99;

        T1 = Transform2D(p, PI/3.0);
        myRobot = DiffDrive(T1, 0.5, 0.1);

        pose = myRobot.pose();
        ASSERT_FLOAT_EQ( pose.x, -25.1);
        ASSERT_FLOAT_EQ( pose.y, 99.99);
        ASSERT_FLOAT_EQ( pose.theta, PI/3.0);
}

TEST(rigid2D, updateOdometry) {
        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 0.5, 0.1);

	// Start the zero angle and move 0 radians
	myRobot.updateOdometry(0.0, 0.0);	
        
	ASSERT_FLOAT_EQ(myRobot.encoder_left, 0.0);
        ASSERT_FLOAT_EQ(myRobot.encoder_right, 0.0);

	// Start the zero angle and move Pi radians on each wheel
        // Moves the robot staight forward
	myRobot.updateOdometry(PI, PI);
	
	ASSERT_FLOAT_EQ(myRobot.encoder_left, PI);
        ASSERT_FLOAT_EQ(myRobot.encoder_right, PI);
		
	geometry_msgs::Pose2D pose = myRobot.pose();
	ASSERT_FLOAT_EQ(pose.x, 0.1 * PI);
        ASSERT_FLOAT_EQ(pose.y, 0.0);
	
	// Rotate in place
	p.x = 0.0;
        p.y = 0.0;
	T1 = Transform2D(p, 0.0);
	myRobot = DiffDrive(T1, 10.0, 10.0);
	myRobot.updateOdometry(-PI, PI);

        ASSERT_FLOAT_EQ(myRobot.encoder_left, -PI);
        ASSERT_FLOAT_EQ(myRobot.encoder_right, PI);

        pose = myRobot.pose();
        ASSERT_FLOAT_EQ(pose.x, 0.0);
        ASSERT_FLOAT_EQ(pose.y, 0.0);

	// Another rotation in place	
	p.x = 54.0;
        p.y = 89.1;
        T1 = Transform2D(p, 0.0);
        myRobot = DiffDrive(T1, 10.0, 10.0);
        myRobot.updateOdometry(-PI, PI);

        ASSERT_FLOAT_EQ(myRobot.encoder_left, -PI);
        ASSERT_FLOAT_EQ(myRobot.encoder_right, PI);

        pose = myRobot.pose();
        ASSERT_FLOAT_EQ(pose.x, 54.0);
        ASSERT_FLOAT_EQ(pose.y, 89.1);

	// Translate and rotate
	p.x = 0.0;
        p.y = 0.0;
        T1 = Transform2D(p, 0.0);
        myRobot = DiffDrive(T1, 0.5, 0.1);
        myRobot.updateOdometry(-1.0, 2.0);

        pose = myRobot.pose();
        ASSERT_TRUE( almost_equal(pose.x, 0.0470, 0.001) );
        ASSERT_TRUE( almost_equal(pose.y, 0.0145, 0.001) );
	ASSERT_TRUE( almost_equal(pose.theta, 0.6, 0.001) );

	p.x = 34.1;
        p.y = 85.0;
        T1 = Transform2D(p, 0.0);
        myRobot = DiffDrive(T1, 0.5, 0.1);
        myRobot.updateOdometry(-1.0, 2.0);

        pose = myRobot.pose();
        ASSERT_TRUE( almost_equal(pose.x, 34.1470, 0.001) );
        ASSERT_TRUE( almost_equal(pose.y, 85.0145, 0.001) );
        ASSERT_TRUE( almost_equal(pose.theta, 0.6, 0.001) );
}


TEST(rigid2D, feedForward) {
        using namespace rigid2d;

        Vector2D p;
        p.x = 0.0;
        p.y = 0.0;
        Transform2D T1 = Transform2D(p, 0.0);
        DiffDrive myRobot = DiffDrive(T1, 0.5, 0.1);
	
	// Translate only
	Twist2D tw = Twist2D(0, -45.6, 0.0);

	myRobot.feedforward(tw);		
		
	geometry_msgs::Pose2D pose = myRobot.pose();
        ASSERT_FLOAT_EQ(pose.x, -45.6);
        ASSERT_FLOAT_EQ(pose.y, 0.0);

	// Rotate only
	p.x = 0.0;
        p.y = 0.0;
        T1 = Transform2D(p, 0.0);
        myRobot = DiffDrive(T1, 0.5, 0.1);	
	tw = Twist2D(3.14/5.0, 0.0, 0.0);
        myRobot.feedforward(tw);
		
	pose = myRobot.pose();
	ASSERT_FLOAT_EQ(pose.theta, 3.14/5.0);

	// Translate and rotate
	p.x = 0.0;
        p.y = 0.0;
        T1 = Transform2D(p, 0.0);
        myRobot = DiffDrive(T1, 0.5, 0.1);          
        
	WheelVelocities vel;
	vel.left = -1.0;
	vel.right = 2.0;
	
	myRobot.feedforward(myRobot.wheelsToTwist(vel));

        pose = myRobot.pose();
        ASSERT_TRUE( almost_equal(pose.x, 0.0470, 0.001) );
	ASSERT_TRUE( almost_equal(pose.y, 0.0145, 0.001) );
	ASSERT_TRUE( almost_equal(pose.theta, 0.6, 0.001) );
}

TEST(rigid2D, nextWayPoint_Rotate_Only) {
	using namespace rigid2d;

        Vector2D p;
        p.x = 1.0;
        p.y = 0.0;
        Transform2D T_world_robot = Transform2D(p, PI/2.0);     
        
        p.x = 1.00001;
        p.y = 2.0;
        
        // The robot should already be at the desired angle
        std::vector<Vector2D> points;
	WayPoints myWpt = WayPoints(points);
        Twist2D nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);
        
        ASSERT_TRUE( almost_equal(nextTwist.w, 0.0, 0.0001) );

        // Make the robot rotate 180 degrees
        p.x = 1.0;
        p.y = 0.0;
        T_world_robot = Transform2D(p, 0.0);
        
        p.x = -2.0;
        p.y = 0.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);
        
        ASSERT_FLOAT_EQ(nextTwist.w, PI);

	p.x = 1.0;
        p.y = 1.0;
        T_world_robot = Transform2D(p, PI/2.0);

        p.x = -2.0;
        p.y = 1.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ(nextTwist.w, PI/2);
	
	p.x = 1.0;
        p.y = 1.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 2.0;
        p.y = 2.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ(nextTwist.w, PI/4);

	p.x = 1.0;
        p.y = 1.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 2.0;
        p.y = 4.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_TRUE(almost_equal(nextTwist.w, 1.249045, 0.0001));
}


/* Test computing the error in the angle
 */
TEST(rigid2D, nextWayPoint_Rotate_Only_2) {
        using namespace rigid2d;

	Vector2D p;
	p.x = 1.0;
	p.y = 0.0;
	Transform2D T_world_robot = Transform2D(p, PI/2.0);	
	
	p.x = 1.00001;
	p.y = 2.0;
	
	// The robot should already be at the desired angle
	std::vector<Vector2D> points;
	WayPoints myWpt = WayPoints(points);
	Twist2D nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);
	
	ASSERT_TRUE( almost_equal(nextTwist.w, 0.0, 0.0001) );

	// Make the robot rotate 180 degrees
	p.x = 1.0;
        p.y = 0.0;
        T_world_robot = Transform2D(p, 0.0);
	
	p.x = -2.0;
	p.y = 0.0;
	nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);
	
	ASSERT_FLOAT_EQ( nextTwist.w, PI );

	// Make the robot go to a negative angle
	p.x = 1.0;
        p.y = -4.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 1.0;
        p.y = -8.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ( nextTwist.w, -PI/2 );
	
	// Make the robot go to (-3/4)PI
	p.x = 2.0;
        p.y = 0.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 0.0;
        p.y = -2.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ( nextTwist.w, (-3/4.0)*PI );
	
	// Same as above but start at a positive angle
	p.x = 2.0;
        p.y = 0.0;
        T_world_robot = Transform2D(p, PI/2);

        p.x = 0.0;
        p.y = -2.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ( nextTwist.w, (-3/4.0)*PI - (PI/2) );


	// Make the robot go to (3/4)PI
	p.x = 2.0;
        p.y = 2.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 4.0;
        p.y = -4.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ( nextTwist.w, -1 * ( (PI/2.0) - 0.32175055) );	
	

	// Start the robot in another quadrant
	p.x = -2.0;
        p.y = -2.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 4.0;
        p.y = -4.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

	ASSERT_FLOAT_EQ( nextTwist.w, -1 * ( (PI/2.0) - 1.2490457724) ); 

	p.x = -2.0;
        p.y = -2.0;
        T_world_robot = Transform2D(p, -PI/2);

        p.x = 4.0;
        p.y = -4.0;
        nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

	ASSERT_FLOAT_EQ( nextTwist.w, 1.2490457724);
}


/* Test computing the translational error
 */
TEST(rigid2D, nextWayPoint_Translate_Only) {
        using namespace rigid2d;

        Vector2D p;
        p.x = 1.0;
        p.y = 0.0;
        Transform2D T_world_robot = Transform2D(p, PI/2.0);

        p.x = 1.0;
        p.y = 200.0;

	std::vector<Vector2D> points;
	WayPoints myWpt = WayPoints(points);
				
	// Robot is at the right angle
	// Returns the twist in the robot's frame?? or world frame?
	Twist2D nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ(nextTwist.w, 0.0); 
	
	// "Forward" is in the robot's x direction
	ASSERT_FLOAT_EQ(nextTwist.dx, 200.0);
	ASSERT_FLOAT_EQ(nextTwist.dy, 0.0);

	p.x = 1.0;
        p.y = 4.0;
        T_world_robot = Transform2D(p, 0.0);

        p.x = 20.0;
        p.y = 4.0;
	
	nextTwist = myWpt.computeNextWayPoint(T_world_robot, p);

        ASSERT_FLOAT_EQ(nextTwist.w, 0.0);
        // "Forward" is in the robot's x direction
        ASSERT_FLOAT_EQ(nextTwist.dx, 19.0);
        ASSERT_FLOAT_EQ(nextTwist.dy, 0.0);
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){


	testing::InitGoogleTest(&argc, argv);
	/*
	   ros::init(argc, argv, "tester");
	   ros::NodeHandle nh;
	   */

	return RUN_ALL_TESTS();
}
