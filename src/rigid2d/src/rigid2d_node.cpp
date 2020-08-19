/* Implement
 */
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <cmath>

/*
char* lookupInputFileName(int testSet) {
	// /home/peter/Desktop/Turtlebot/catkin_ws/src/rigid2d/test  
	return "test/inputs/testX.json";
}

* Describe
 *
char* lookupOutputFileName(int testSet) {
	
	return "test/outputs/testX.json";
}
*/


/* Describe this here
 */
rigid2d::Transform2D parseJSON(Json::Value frame) {
	double x = std::stod( frame["x"].asString() );
        double y = std::stod( frame["y"].asString() );
        
	// double theta = std::stod( frame["theta"].asString() );
	
	double cTheta = std::stod( frame["cTheta"].asString() );
	double sTheta = std::stod( frame["sTheta"].asString() );

	rigid2d::Vector2D vector;
	vector.x = x;
	vector.y = y;
	
	return rigid2d::Transform2D(vector, cTheta, sTheta);
}

/* I used Eigen to generate some of the test cases. I also am not
 * sure I had more than 4/5 decimal places in the manual calculations. 
 * So, I have some numerical error for the test cases. I use this
 * method to check if two Transforms2D are equal 
 */
bool isEqualForTesting(const rigid2d::Transform2D &lhs, const rigid2d::Transform2D& rhs) {

	  using namespace rigid2d;
	  double epsilon = 0.1;
          if (almost_equal(lhs.vector.x, rhs.vector.x, epsilon) && almost_equal(lhs.vector.y, rhs.vector.y, epsilon) &&
                almost_equal(lhs.cTheta, rhs.cTheta, epsilon) && (almost_equal(lhs.sTheta, rhs.sTheta, epsilon)) ) {

         	return true;
          }

          return false;
}

/* Describe 
 */ 
bool isEqualForTesting(const rigid2d::Vector2D &lhs, const rigid2d::Vector2D &rhs) {
		
	 using namespace rigid2d;
	 double epsilon = 0.1;
                if ( (almost_equal(lhs.x, rhs.x, epsilon) ) && (almost_equal(lhs.y, rhs.y, epsilon) ) ) {
                        return true;
                }

                return false;
}

/* Describe 
 */
bool isEqualForTesting(rigid2d::Twist2D twist_c_correct, rigid2d::Twist2D twist_c) {

	using namespace rigid2d;
         double epsilon = 0.1;
                if ( (almost_equal(twist_c_correct.w, twist_c.w, epsilon) ) && (almost_equal(twist_c_correct.dx, twist_c.dx, epsilon) )
		  && (twist_c_correct.dy, twist_c.dy, epsilon) ) {
                        return true;
                }

                return false;
}


/* Describe 
 */
rigid2d::Vector2D jsonToVector(Json::Value vector) {
	
	double x = std::stod( vector["x"].asString() );
        double y = std::stod( vector["y"].asString() );
		
	rigid2d::Vector2D v;
	v.x = x;
	v.y = y;

	return v;
}

/* Describe this method 
 */
rigid2d::Twist2D jsonToTwist(Json::Value twist) {

	double w = std::stod( twist["w"].asString() );
	double dx = std::stod( twist["dx"].asString() );
	double dy = std::stod( twist["dy"].asString() );
		
	return rigid2d::Twist2D(w, dx, dy);
}


/* Given T_ab and T_bc, compute and output T_ab, T_ba, T_bc, T_cb, T_ac, T_ca
 */
rigid2d::Transform2D testTransforms(rigid2d::Transform2D T_ab, rigid2d::Transform2D T_bc, char outputFileName[] ) {
	
	// Compute the new Transforms
	rigid2d::Transform2D T_ac = T_ab * T_bc;
	
	rigid2d::Transform2D T_ca = T_ac.inv();	

	rigid2d::Transform2D T_ba = T_ab.inv();

	rigid2d::Transform2D T_cb = T_bc.inv();
	
	// Read the json file of labels
	std::ifstream ifs(outputFileName);
        Json::Reader reader;
        Json::Value obj;
        reader.parse(ifs, obj);
	
	// Read in the transforms from the output file
	rigid2d::Transform2D T_ab_correct = parseJSON( obj["T_ab"] );
	rigid2d::Transform2D T_bc_correct = parseJSON( obj["T_bc"] );
	rigid2d::Transform2D T_ac_correct = parseJSON( obj["T_ac"] );
        rigid2d::Transform2D T_ba_correct = parseJSON( obj["T_ba"] );
        rigid2d::Transform2D T_cb_correct = parseJSON( obj["T_cb"] );
        rigid2d::Transform2D T_ca_correct = parseJSON( obj["T_ca"] );

	std::cout.precision(17);

	if( !isEqualForTesting( T_ab, T_ab_correct) ) {
		std::cout << "Frame Conversion Error: The computed T_ab is incorrect" << std::endl;
		std::cout << "The computed T_ab is " << T_ab << std::endl;
		std::cout << "The desired T_ab is " << T_ab_correct << std::endl;
	}
	
	if ( !isEqualForTesting(T_ba, T_ba_correct) ) {
		std::cout << "Frame Conversion Error: The computed T_ba is incorrect" << std::endl;
		std::cout << "The computed T_ba is " << T_ba << std::endl;
                std::cout << "The desired T_ba is " << T_ba_correct << std::endl;
	}
	if ( !isEqualForTesting(T_bc, T_bc_correct) ) {
		std::cout << "Frame Conversion Error: The computed T_bc is incorrect" << std::endl;
		std::cout << "The computed T_bc is " << T_bc << std::endl;
                std::cout << "The desired T_bc is " << T_bc_correct << std::endl;
	}
	
	if ( !isEqualForTesting( T_cb, T_cb_correct) ) {
		std::cout << "Frame Conversion Error: The computed T_cb is incorrect" << std::endl;
		std::cout << "The computed T_cb is " << T_cb << std::endl;
                std::cout << "The desired T_cb is " << T_cb_correct << std::endl;
		/*	
		std::cout << T_cb.getCTheta() << " " << T_cb.getSTheta() << std::endl;
		std::cout << T_cb_correct.getCTheta() << " " << T_cb_correct.getSTheta() << std::endl;
		*/	
	}
	
	if ( !isEqualForTesting( T_ac, T_ac_correct) ) {
		std::cout << "Frame Conversion Error: The computed T_ac is incorrect" << std::endl;	
		std::cout << "The computed T_ac is " << T_ac << std::endl;
                std::cout << "The desired T_ac is " << T_ac_correct << std::endl;
	}
	
	if ( !isEqualForTesting( T_ca, T_ca_correct) ) {
                std::cout << "Frame Conversion Error: The computed T_ca is incorrect" << std::endl; 
		std::cout << "The computed T_ca is " << T_ca << std::endl;
                std::cout << "The desired T_ca is " << T_ca_correct << std::endl;
		
		//std::cout << T_ca.getCTheta() << " " << T_ca.getSTheta() << std::endl;
                //std::cout << T_ca_correct.getCTheta() << " " << T_ca_correct.getSTheta() << std::endl;	
	
		std::cout << std::endl;
		std::cout << "computed x is " << T_ca.vector.x << "\n" << "correct x is " << T_ca_correct.vector.x << " \n" << std::endl;
			

		std::cout << "std::fabs of the x values is " << std::fabs(T_ca.vector.x - T_ca_correct.vector.x ) << std::endl;
	}

	return T_ac;
}

/* Describe 
 */
bool testUserInput(char inputFileName[], char outputFileName[]) {
	
	// Re-direct stdin to the input file
        std::ifstream in(inputFileName);
        //std::streambuf *cinbuf = std::cin.rdbuf(); //save old buf
        std::cin.rdbuf(in.rdbuf()); //redirect std::cin to in.txt!

	// Read the input files 
        rigid2d::Transform2D T_ab;
        std::cin >> T_ab;

        rigid2d::Transform2D T_bc;
        std::cin >> T_bc;
		
        // Read the json file of labels
	//std::cout << inputFileName << std::endl;
        std::ifstream ifs(outputFileName);
        Json::Reader reader;
        Json::Value obj;
        reader.parse(ifs, obj);

	
	rigid2d::Transform2D T_ab_correct = parseJSON( obj["T_ab"] );
        rigid2d::Transform2D T_bc_correct = parseJSON( obj["T_bc"] );
	
	// Make sure inputs and labels match	
	using namespace rigid2d;
	if ( ( almost_equal(T_ab_correct.getX(), T_ab.getX(), 0.01) && ( almost_equal(T_ab_correct.getCTheta(), T_ab.getCTheta(), 0.01) ) &&
	    (almost_equal(T_ab_correct.getCTheta(), T_ab.getCTheta(), 0.01) ) && (almost_equal(T_ab_correct.getSTheta(), T_ab.getSTheta(), 0.01)) ) ) {
		
		std::cout << "Operator overload methods input/processed data incorrectly" << std::endl;
		return false;
	}
	if ( ( almost_equal(T_bc_correct.getX(), T_bc.getX(), 0.01) && ( almost_equal(T_bc_correct.getCTheta(), T_bc.getCTheta(), 0.01) ) &&
            (almost_equal(T_bc_correct.getCTheta(), T_bc.getCTheta(), 0.01) ) && (almost_equal(T_bc_correct.getSTheta(), T_bc.getSTheta(), 0.01)) ) ) { 
	
		std::cout << "Operator overload methods input/processed data incorrectly" << std::endl;
		return false;
	}
	

	return true;	
}

/* Describe 
 */
bool convertVectors(char inputFileName[], char outputFileName[]) {
	
	using namespace rigid2d;
	// Read the json file of labels
        std::ifstream ifs(outputFileName);
        Json::Reader reader;
        Json::Value obj;
        reader.parse(ifs, obj);

        // Read in the transforms from the output file
        Transform2D T_ab = parseJSON( obj["T_ab"] );
        Transform2D T_bc = parseJSON( obj["T_bc"] );

	// Read in the correct vectors from the JSON files
	Vector2D V_a_correct = jsonToVector( obj["V_a"] );  
	Vector2D V_b_correct = jsonToVector( obj["V_b"] );
	Vector2D V_c_correct = jsonToVector( obj["V_c"] );



	// Read the json file of inputs
        std::ifstream ifs2(inputFileName);
        reader.parse(ifs2, obj);

	Vector2D V_a;
	char frame = obj["vector"]["frame"].asString()[0];
        V_a.x = std::stod( obj["vector"]["x"].asString() );
	V_a.y = std::stod( obj["vector"]["y"].asString() );

	// Convert the vector using the 2D homogenous transformation
	// to frame A, B, C
	// Assuming V1 is 
	Vector2D V_b = (T_ab.inv())(V_a);      
	
	Vector2D V_c = ((T_ab * T_bc).inv())(V_a);

	// Check that the new vector in A, B, C
	// match the computed vectors
	bool isCorrect = true;	
	if ( !isEqualForTesting( V_a_correct, V_a) ) { 
		std::cout << "Incorrect frame transformation: vector in A frame is incorrect" << std::endl;
		isCorrect = false;
	
		std::cout << "The computed V_a is " << V_a << " and the real V_a is " << V_a_correct << std::endl;
	}
	if ( !isEqualForTesting(V_b_correct, V_b) ) {
		std::cout << "Incorrect frame transformation: vector in B frame is incorrect" << std::endl;
		isCorrect = false;

		std::cout << "The computed V_b is " << V_b << " and the real V_b is " << V_b_correct << std::endl;
	}
	if ( !isEqualForTesting(V_c_correct, V_c) ) {
		std::cout << "Incorrect frame transformation: vector in C frame is incorrect" << std::endl;	
		isCorrect = false;

		std::cout << "The computed V_c is " << V_c << " and the real V_c is " << V_c_correct << std::endl;
	}


	return isCorrect;
}


/* Describe this method
 */
bool convertTwists(char inputFileName[], char outputFileName[]) {

	using namespace rigid2d;
        // Read the json file of labels
        std::ifstream ifs(inputFileName);
        Json::Reader reader;
        Json::Value obj;
        reader.parse(ifs, obj);

	// Read in the twists from the input file 
	Twist2D twist_c = jsonToTwist(obj["twist_c"]);

	// Read in the json file of labels
        // Read in the correct, label twist from the output file
        std::ifstream ifs2(outputFileName);
	reader.parse(ifs2, obj);
	
	Transform2D T_ab = parseJSON( obj["T_ab"] );
        Transform2D T_bc = parseJSON( obj["T_bc"] );

	Twist2D twist_a_correct = jsonToTwist(obj["twist_a"]);
	Twist2D twist_b_correct = jsonToTwist(obj["twist_b"]);
	Twist2D twist_c_correct = jsonToTwist(obj["twist_c"]);
	
	// Use the adjoint to convert the twists from one frame to another
	Twist2D twist_b = T_bc(twist_c);

	Twist2D twist_a = (T_ab * T_bc)(twist_c);
	
	//Twist2D twist_a = (T_ab)(twist_b);

	bool isCorrect = true;
	if ( !isEqualForTesting(twist_c_correct, twist_c) ) {
                std::cout << "\nIncorrect twist transformation: twist in the C frame is incorrect" << std::endl;
                std::cout << "The computed twist_c is " << twist_c << " and the real twist_c is " << twist_c_correct << std::endl;
        	isCorrect = false;
	}
	if ( !isEqualForTesting(twist_b_correct, twist_b) ) { 
		std::cout << "\nIncorrect twist transformation: twist in the B frame is incorrect" << std::endl;
                std::cout << "The computed twist_b is " << twist_c << " and the real twist_b is " << twist_c_correct << std::endl;
                isCorrect = false;
	}	
	if ( !isEqualForTesting(twist_a_correct, twist_a) ) {
                std::cout << "\nIncorrect twist transformation: twist in the A frame is incorrect" << std::endl;
                std::cout << "The computed twist_a is " << twist_a << " and the real twist_a is " << twist_a_correct << std::endl;
                isCorrect = false;
        }

	// std::cout << twist_a_correct << std::endl << twist_a << std::endl;

	return isCorrect;
}




/* The input will be a scalar indicating which testing files to use
 * So if argv[1] = N, then we will use the Nth set of testing files
 */
int main(int argc, char *argv[]) {
	
	if (argc < 2) {
		std::cout << "Incorrect testing set command line argument" << std::endl;
                return -1;
	}
	
	// The output files are at test/output/testN.txt	
	// The input files are at test/input/testN.txt
	
	//std::string pkg_name = "rigid2d"; 
	//ros::package::getPath(pkg_name); 		
	// FIX ME - look up the file path with getPath
	// http://docs.ros.org/indigo/api/roslib/html/c++/namespaceros_1_1package.html#ae9470dd201aa4e66abb833e710d812a4
	// /home/peter/Desktop/Turtlebot/catkin_ws/src/rigid2d/test	
	
	char inputFileName[] = "test/inputs/testX.json";
	char outputFileName[] = "test/outputs/testX.json";
	int testSet = atof( argv[1] );
	/*
	char inputFileName[] = lookupInputFileName(testSet);
        char outputFileName[] = lookupOutputFileName(testSet);
	*/

	// Fix to reflect number of tests I write
	int maxSetNum = 10;

	if ( (testSet < 0) || (testSet > maxSetNum) ) {
		// Print a description of what the command should look like
		std::cout << "Incorrect testing set command line argument" << std::endl;
		return -1;
	}
	

	// The command line arg indicates which input/label files to test with
	// If it is 0, we will test the user's input by re-directing stdin
	if (testSet == 0) {
		char inputFileName[] = "test/inputs/testX.txt";
		inputFileName[16] = *argv[1];	
		outputFileName[17] = *argv[1];
		
		// Run tests on inputting/reading in frames	
		testUserInput(inputFileName, outputFileName);
		return 0;
	}
	else {
		inputFileName[16] = *argv[1];
		outputFileName[17] = *argv[1];	
	}
	
	//std::cout << inputFileName << "\n";	
        // Read the json file of labels
        std::ifstream ifs(inputFileName);
        Json::Reader reader;
        Json::Value obj;
        reader.parse(ifs, obj);

        // Read in the transforms from the output file
        rigid2d::Transform2D T_ab = parseJSON( obj["T_ab"] );
        rigid2d::Transform2D T_bc = parseJSON( obj["T_bc"] );
	
	// Run tests on the inputted data
	testTransforms(T_ab, T_bc, outputFileName);		
		
	
	if (testSet == 1) {
	
		convertTwists(inputFileName, outputFileName);
	}
	

	if (testSet > 4) {
		double x = -1.0;
		double y = -1.0;
	
		char frame = 'A';
		
		convertVectors(inputFileName, outputFileName);

		//convertTwists(inputFileName, outputFileName);


		/*
		std::cout << "Enter a vector v \n";	

		std::cout << "Enter {A, B, C} indicating the frame in which the vector is defined \n";
		std::cin >> frame;

		std::cout << "Enter a vector's x component \n";
		std::cin >> x;
	
		std::cout << "Enter a vector's y component \n";
		std::cin >> y;

		rigid2d::Vector2D V1;
		V1.x = x;
		V1.y = y;	
	
		std::cout << "The vector is ";
		std::cout << "(" << V1.x << ", " << V1.y << ") and is defined in the " << frame << " frame \n";

		// Output the 	
		*/	 
	}


	return 1;
}
