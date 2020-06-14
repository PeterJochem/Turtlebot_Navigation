/* Implement
 */
#include "rigid2d.hpp"
#include <iostream>

int main() {
	
	rigid2d::Vector2D myVector;
	myVector.x = 1;
	myVector.y = 2;
	
	std::cout << myVector;

	rigid2d::Transform2D myTransform = rigid2d::Transform2D(1.0, 5.0, 0.0);	
	
	std::cout << myTransform;

	return 1;
}
