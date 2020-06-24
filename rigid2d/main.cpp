/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>


/* Given T_ab and T_bc, compute and output T_ab, T_ba, T_bc, T_cb, T_ac, T_ca
 */
rigid2d::Transform2D testTransforms(rigid2d::Transform2D T_ab, rigid2d::Transform2D T_bc, char outputFileName[] ) {
	
	// Compute the new Transforms
	rigid2d::Transform2D T_ac = T_ab * T_bc;
	
	rigid2d::Transform2D T_ba = T_ab.inv();

	rigid2d::Transform2D T_cb = T_bc.inv();

	rigid2d::Transform2D T_ca = T_ac.inv();
	
	// Read in the transforms from the output file
	rigid2d::Transform2D T_ab_correct;
        rigid2d::Transform2D T_bc_correct;
	rigid2d::Transform2D T_ac_correct;
        rigid2d::Transform2D T_ba_correct;
        rigid2d::Transform2D T_cb_correct;
        rigid2d::Transform2D T_ca_correct;
	
	// Read in each transform from the test/outputs file 	
	// Re-direct stdin to the input file
        std::ifstream in(outputFileName);
        std::streambuf *cinbuf = std::cin.rdbuf(); //save old buf
        std::cin.rdbuf(in.rdbuf()); //redirect std::cin to in.txt!

	// Read in each transform - for now just read in T_ab and T_bc
	// Read in this order: T_ab, T_ba, T_bc, T_cb, T_ac, T_ca
	std::cin >> T_ab_correct;
	std::cin >> T_ba_correct;	
	std::cin >> T_bc_correct;			
	std::cin >> T_cb_correct;	
	std::cin >> T_ac_correct;
	std::cin >> T_ca_correct;

	T_ca = T_ac_correct.inv();

	// T_ab, T_ba, T_bc, T_cb, T_ac, T_ca
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
	}
	

	return T_ac;
}




/* The input will be a scalar indicating which testing files to use
 * So if argv[3] = N, then we will use the Nth set of testing files
 */
int main(int argc, char *argv[]) {
	
	// For testing, use this
	double pi = 3.14159265358979;
	
	// The output files are at test/output/testN.txt	
	// The input files are at test/input/testN.txt
		
	char inputFileName[] = "test/inputs/testX.txt";
	int testSet = atof( argv[1] );
		
	inputFileName[16] = *argv[1];
	//std::cout << inputFileName << std::endl;  
	
	
	// Re-direct stdin to the input file
	std::ifstream in(inputFileName);
	std::streambuf *cinbuf = std::cin.rdbuf(); //save old buf
    	std::cin.rdbuf(in.rdbuf()); //redirect std::cin to in.txt!
	
	rigid2d::Transform2D T_ab;
	std::cin >> T_ab;	

	rigid2d::Transform2D T_bc;
        std::cin >> T_bc;
	
	// std::cout << T_ab << std::endl;	

	testTransforms(T_ab, T_bc, "test/outputs/test1.txt");
			


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
