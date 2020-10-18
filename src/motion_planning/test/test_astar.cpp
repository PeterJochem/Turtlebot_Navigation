#include "motion_planning/A_Star_Planner.hpp"
#include <gtest/gtest.h>
#include <string>
#include <fstream>
#include <sstream>

std::tuple<std::vector<int>, int, int> readGrid(std::string fileName) {
	using namespace std;

	ifstream myFile(fileName);
	
	string line;	
	int start_x, start_y, goal_x, goal_y;
	
	std::getline(myFile, line);
	stringstream ss(line);
      	
	// Read in the start position and goal positions	
	ss >> start_x;
	ss.ignore(); 
	ss >> start_y;
       	ss.ignore();	
	ss >> goal_x;
       	ss.ignore();		
	ss >> goal_y;
	
	int width = 0;
	int height = 0;

	vector<int> grid = vector<int>();
	
	// Read in the grid data itself
	while(std::getline(myFile, line)) {
		
		width = 0;
		stringstream ss(line);
		int nextProb = 0;
		while (ss >> nextProb) {
			
			nextProb = (1 - nextProb) * 100;

			grid.push_back(nextProb);
			ss.ignore();
			width++;
		}

		height++;
	}
	
	return {grid, width, height};	
}

TEST(testSuiteName, simplePath1) {	
	using namespace std;

	// Must use the absolute path because where the binary runs is not in this folder with the source file
	string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid1.csv";
	auto[grid, width, height] = readGrid(fileName); 

	// int* map, int height, int width, int resolution)
	A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);
	
	// start_x, start_y, goal_x, goal_y	
	myPlanner.setGoal(0.0, 0.0, 7.0, 7.0);
	std::vector<std::tuple<double, double>> path = myPlanner.plan();
}

TEST(testSuiteName, simplePath2) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
}


// FINISH IT!
TEST(testSuiteName, goalIsOutsideMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        //myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        //std::vector<std::tuple<double, double>> path = myPlanner.plan();
}

// FINISH IT!
TEST(testSuiteName, startPointIsOutsideMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        //myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        //std::vector<std::tuple<double, double>> path = myPlanner.plan();
}





int main(int argc, char **argv){

        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

