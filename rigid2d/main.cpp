/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>


/* Given T_ab and T_bc, compute T_ac
 */
rigid2d::Transform2D testTransforms(rigid2d::Transform2D T_ab, rigid2d::Transform2D T_bc) {
	
	rigid2d::Transform2D T_ac = T_ab * T_bc;
	std::cout << T_ac << std::endl;	

	// Write T_ac to the output file

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
		
	std::cout << atof( argv[1] ) << std::endl;
	
	char inputFileName[] = "test/inputs/testX.txt";
	int testSet = atof( argv[1] );
		
	inputFileName[16] = *argv[1];
	std::cout << inputFileName << std::endl;  
	
	// Re-direct stdin to the input file
	std::ifstream in(inputFileName);
	std::streambuf *cinbuf = std::cin.rdbuf(); //save old buf
    	std::cin.rdbuf(in.rdbuf()); //redirect std::cin to in.txt!
	
	double i = -1;
	std::cin >> i;
	std::cout << i << std::endl;

	rigid2d::Transform2D T_ab;
	std::cout << "Enter T_ab \n";	
	std::cin >> T_ab;	

	rigid2d::Transform2D T_bc;
        std::cout << "Enter T_bc \n";
        std::cin >> T_bc;
	

	testTransforms(T_ab, T_bc);
			
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
