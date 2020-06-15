/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>

int main() {
	
	rigid2d::Vector2D myVector1;
	myVector1.x = 100.0;
	myVector1.y = 100.0;
	
	rigid2d::Vector2D myVector2;
        myVector2.x = -400;
        myVector2.y = -2;

	// Test each of the three constructors
	// rigid2d::Transform2D tf1 = rigid2d::Transform2D();	
	// rigid2d::Transform2D tf2 = rigid2d::Transform2D(myVector2);
	rigid2d::Transform2D tf3 = rigid2d::Transform2D(3.14);

	
	std::cout << tf3(myVector1);
	// std::cout << tf3.getTf() << std::endl; 
	

		

	return 1;
}
