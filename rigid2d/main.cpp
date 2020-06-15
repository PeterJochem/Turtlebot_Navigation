/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>

int main() {
	
	rigid2d::Vector2D myVector;
	myVector.x = 400;
	myVector.y = 2;
	
	std::cout << myVector;

	rigid2d::Transform2D myTransform = rigid2d::Transform2D(400.0, 5.0, 90.0);	
	
	std::cout << myTransform;

	std::cout << myTransform.getTf() << std::endl; 

	return 1;
}
