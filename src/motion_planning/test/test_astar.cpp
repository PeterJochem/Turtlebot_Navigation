#include "motion_planning/A_Star_Planner.hpp"
#include <gtest/gtest.h>


TEST(myName, myOtherName) {
	
	std::cout << "WE ARE HERE" << std::endl;
	
	int map [5] = {16, 2, 77, 40, 12071}; 

	A_Star_Planner myPlanner = A_Star_Planner(nullptr, 1, 1, 1);
}


int main(int argc, char **argv){

        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

