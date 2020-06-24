/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>
#include <iostream>
#include <jsoncpp/json/json.h>


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
	
	if( T_ab != T_ab_correct ) {
		std::cout << "Frame Conversion Error: The computed T_ab is incorrect" << std::endl;
		std::cout << "The computed T_ab is " << T_ab << std::endl;
		std::cout << "The desired T_ab is " << T_ab_correct << std::endl;
	}
	
	if ( T_ba != T_ba_correct ) {
		std::cout << "Frame Conversion Error: The computed T_ba is incorrect" << std::endl;
		std::cout << "The computed T_ba is " << T_ba << std::endl;
                std::cout << "The desired T_ba is " << T_ba_correct << std::endl;
	}
	if ( T_bc != T_bc_correct ) {
		std::cout << "Frame Conversion Error: The computed T_bc is incorrect" << std::endl;
		std::cout << "The computed T_bc is " << T_bc << std::endl;
                std::cout << "The desired T_bc is " << T_bc_correct << std::endl;
	}
	
	if ( T_cb != T_cb_correct ) {
		std::cout << "Frame Conversion Error: The computed T_cb is incorrect" << std::endl;
		std::cout << "The computed T_cb is " << T_cb << std::endl;
                std::cout << "The desired T_cb is " << T_cb_correct << std::endl;
		/*	
		std::cout << T_cb.getCTheta() << " " << T_cb.getSTheta() << std::endl;
		std::cout << T_cb_correct.getCTheta() << " " << T_cb_correct.getSTheta() << std::endl;
		*/	
	}
	
	if ( T_ac != T_ac_correct ) {
		std::cout << "Frame Conversion Error: The computed T_ac is incorrect" << std::endl;	
		std::cout << "The computed T_ac is " << T_ac << std::endl;
                std::cout << "The desired T_ac is " << T_ac_correct << std::endl;
	}
	
	if ( T_ca != T_ca_correct ) {
                std::cout << "Frame Conversion Error: The computed T_ca is incorrect" << std::endl; 
		std::cout << "The computed T_ca is " << T_ca << std::endl;
                std::cout << "The desired T_ca is " << T_ca_correct << std::endl;
		
		std::cout << T_ca.getCTheta() << " " << T_ca.getSTheta() << std::endl;
                std::cout << T_ca_correct.getCTheta() << " " << T_ca_correct.getSTheta() << std::endl;	
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


/* The input will be a scalar indicating which testing files to use
 * So if argv[3] = N, then we will use the Nth set of testing files
 */
int main(int argc, char *argv[]) {
	
	if ( argc < 2) {
		std::cout << "Incorrect testing set command line argument" << std::endl;
                return -1;
	}
	
	// The output files are at test/output/testN.txt	
	// The input files are at test/input/testN.txt
		
	char inputFileName[] = "test/inputs/testX.json";
	char outputFileName[] = "test/outputs/testX.json";
	int testSet = atof( argv[1] );

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
	

	/*
	double x = -1.0;
	double y = -1.0;
	
	char frame = 'A';

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

	return 1;
}
