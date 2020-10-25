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
	double resolution;

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
	ss.ignore();
        ss >> resolution;	

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

	ASSERT_EQ(path.size(), 11);
}

TEST(testSuiteName, multiplePathsPossible_1) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid3.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();

	ASSERT_EQ(path.size(), 11);
}

TEST(testSuiteName, multiplePathsPossible_2) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid4.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        myPlanner.setGoal(0.0, 0.0, 6.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();

	ASSERT_EQ(path.size(), 14);
}

TEST(testSuiteName, goalIsOutsideMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(0.0, 0.0, 5.0, 7.0), false);
}

TEST(testSuiteName, startPointIsOutsideMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(-20.0, -200.0, 3.0, 2.0), false);
        //std::vector<std::tuple<double, double>> path = myPlanner.plan();
}

TEST(testSuiteName, startPointIsGoalPoint) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(0.0, 0.0, 0.0, 0.0), true);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();

	ASSERT_EQ(path.size(), 1);
}

TEST(testSuiteName, startNodeIsNotFree) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(2.0, 2.0, 0.0, 0.0), false);
}

/* Give an illegal path. Let it fail. Then re-plan */
TEST(testSuiteName, replanning_1) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(2.0, 2.0, 0.0, 0.0), false);

	myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();

        ASSERT_EQ(path.size(), 11);
}

/* Give multipl illegal paths. Let it fail. Then re-plan */
TEST(testSuiteName, replanning_2) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        auto[grid, width, height] = readGrid(fileName);

        // int* map, int height, int width, int resolution)
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(2.0, 2.0, 0.0, 0.0), false);
	ASSERT_EQ(myPlanner.setGoal(2.0, 3.0, 0.0, 0.0), false);


        myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
        ASSERT_EQ(path.size(), 11);
}

/* Update the map and make sure correct path is still found*/
TEST(testSuiteName, updateMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid3.csv";
	auto[grid, width, height] = readGrid(fileName);
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);
	
	fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";	
	tie(grid, width, height) = readGrid(fileName);
	myPlanner.updateMap(grid.data(), width, height, 1);

	// start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(2.0, 2.0, 0.0, 0.0), false);
        ASSERT_EQ(myPlanner.setGoal(2.0, 3.0, 0.0, 0.0), false);

        myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
        ASSERT_EQ(path.size(), 11);
}

/* Update the map and make sure correct path is still found*/
/*
TEST(testSuiteName, updateMap) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid3.csv";
        auto[grid, width, height] = readGrid(fileName);
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 1);

        fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid2.csv";
        tie(grid, width, height) = readGrid(fileName);
        myPlanner.updateMap(grid.data(), width, height, 1);

        // start_x, start_y, goal_x, goal_y
        ASSERT_EQ(myPlanner.setGoal(2.0, 2.0, 0.0, 0.0), false);
        ASSERT_EQ(myPlanner.setGoal(2.0, 3.0, 0.0, 0.0), false);

        myPlanner.setGoal(0.0, 0.0, 3.0, 7.0);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
        ASSERT_EQ(path.size(), 11);
}
*/

/* Plan a path at variable resolution */ 
TEST(testSuiteName, variableResolution_1) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid5.csv";
        auto[grid, width, height] = readGrid(fileName);
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 0.33);
	
	// We set the goal in the map frame 	
        myPlanner.setGoal(0.0, 0.0, 2.8, 2.8);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
        ASSERT_EQ(path.size(), 17);
}

/* Plan a path at variable resolution */
TEST(testSuiteName, variableResolution_2) {
        using namespace std;

        // Must use the absolute path because where the binary runs is not in this folder with the source file
        string fileName = "/home/peter/Desktop/Turtlebot/catkin_ws/src/motion_planning/test/test_grids/grid5.csv";
        auto[grid, width, height] = readGrid(fileName);
        A_Star_Planner myPlanner = A_Star_Planner(grid.data(), height, width, 4.0);

        // We set the goal in the map frame     
        myPlanner.setGoal(0.0, 0.0, 35.8, 35.8);
        std::vector<std::tuple<double, double>> path = myPlanner.plan();
        ASSERT_EQ(path.size(), 17);
}


int main(int argc, char **argv){

        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

