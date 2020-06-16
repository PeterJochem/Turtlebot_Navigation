/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>

int main() {
	
	// For testing, use this
	double pi = 3.14159265358979;

	rigid2d::Vector2D myVector1;
	myVector1.x = 1.0;
	myVector1.y = 0.0;


	rigid2d::Vector2D myVector2;
        myVector2.x = -400.0;
        myVector2.y = -20.0;

	// Test each of the three constructors
	rigid2d::Transform2D tf3; // = rigid2d::Transform2D();	
	
	rigid2d::Transform2D tf2 = rigid2d::Transform2D(myVector2);
	// rigid2d::Transform2D tf3 = rigid2d::Transform2D(0);	
	// std::cout << tf3(myVector1);
	// std::cout << tf3.getTf() << std::endl; 
				
	//std::cin >> tf3;		
	//std::cout << tf3;

	//rigid2d::Twist2D myTwist = rigid2d::Twist2D(0.0, 1.0, 2.0);	
	//std::cout << myTwist;

	//rigid2d::Twist2D myTwist; // = rigid2d::Twist2D(0.0, 1.0, 2.0);   
        //std::cin >> myTwist;	
	
	//std::cout << myTwist;
	
	// Convert the twist from tf3 to the new frame 
	std::cout << tf3(myTwist, tf2);
	



	return 1;
}
