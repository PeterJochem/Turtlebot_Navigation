/** @file
 * @brief Describe  
 *
 * Parameters: 
 *
 * Publishes: 
 *
 * Subscribes: 
 *
 * Services:  */

#include "motion_planning/A_Star_Planner.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include "ros/ros.h"


// See http://neutrofoton.github.io/blog/2016/12/29/c-plus-plus-priority-queue-with-comparator/
/*struct gridCellCompare {
    bool operator()(const gridCell* lhs, const gridCell* rhs) {
        return lhs->estimatedDistance < rhs->estimatedDistance;
    }
}; */


/** @brief Determine if the (x, y) pair in grid coordinates 
 * actually exists on the map
 */
inline bool A_Star_Planner::isLegal(int x, int y) {
	
	if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) {
		return false;
	}
	return true;
}

/** @brief Check if the (x, y) pair in grid coordinates
 *  can be added to the frontier. Have we already added it
 *  to the frontier?
 */
inline bool A_Star_Planner::unexplored(int grid_x, int grid_y) {
	
	int index = computeIndex(grid_x, grid_y);
	return !grid[index].explored;
}

/** @brief Describe me 
 */
inline bool A_Star_Planner::isFree(int grid_x, int grid_y) {
	
	int index = computeIndex(grid_x, grid_y);
        return grid[index].probability < prob_threshold;
}

/** @brief
 *
 */
void A_Star_Planner::expandFrontier() {
	using namespace std;

	if (frontier.size() == 0) {
		return;
	}

	// Pop the top item
	gridCell* currentCell = frontier.top();
	frontier.pop(); // Returns null

	vector<tuple<int, int>> allChildrenCoords = currentCell->generateChildren();

	for (unsigned int i = 0; i < allChildrenCoords.size(); i++) {
		
		auto[nextX, nextY] = allChildrenCoords.at(i); 		
		if (isLegal(nextX, nextY) && unexplored(nextX, nextY) && isFree(nextX, nextY)) {
			// add child to the frontier
			
			gridCell* nextCell = getGridCellPtr(nextX, nextY);
			nextCell->explored = true;
				
			frontier.push(nextCell);
		}	
	}
}


/** @brief Describe me
 */
A_Star_Planner::A_Star_Planner(int* map, int height, int width, int resolution) {
	using namespace std;

	this->map = map;
	grid_height = height;
	grid_width = width;
	grid_resolution = resolution;
	prob_threshold = 50; // Make this a parameter on the ROS server

	// Remember to delete this
	vector<gridCell> grid = vector<gridCell>();
	grid.reserve(grid_width * grid_height);	
	
	for (int i = 0; i < grid_width * grid_height; i++) {
		
		auto[nextX, nextY] = indexToCoords(i);
		grid.push_back(gridCell(nextX, nextY));
	}
}


/** @brief Convert coordinates in the grid to the map frame 
 *  FIX ME - DOXY
 */
std::tuple<double, double> A_Star_Planner::gridToMeters(int grid_x, int grid_y) {

	double x_map = grid_x / grid_resolution; 			
	double y_map = grid_y / grid_resolution;
		
	return {x_map, y_map};
}

/** @brief 
 */
std::tuple<int, int> A_Star_Planner::indexToCoords(int index) {
	
	int nextX = index % grid_width; 
	int nextY = index / grid_width;

	return {nextX, nextY};
}

/** @brief 
 */
gridCell* A_Star_Planner::getGridCellPtr(int nextX, int nextY) {
	
	int index = computeIndex(nextX, nextY);
	return &grid[index];
}

/** @brief 
 */
std::tuple<int, int> A_Star_Planner::metersToGrid(double map_x, double map_y) {
	
        int x_grid = map_x * grid_resolution; // Truncates the fractional part
        int y_grid = map_y * grid_resolution;

        return {x_grid, y_grid};
}

/** @brief Given the goal in the map frame, find the closest grid cell coordinates
 */
std::tuple<int, int> A_Star_Planner::goalToGrid(double map_x, double map_y) {

		
	auto[x_grid, y_grid] = metersToGrid(map_x, map_y);
	
	
	// Check for -grid indexes
        if (x_grid >= grid_width || y_grid >= grid_height) {
                // We are out of bounds
                // std::cout << ""
        }

        return {x_grid, y_grid};
}

/** @brief Given the goal coordinates in grid cells, compute the index
 * of the cell item in the linear data array from the OccupancyGrid 
 * The occupancy grid is in row-major order
 */
int A_Star_Planner::computeIndex(int grid_x, int grid_y) {
	
	return (grid_y * grid_width) + grid_x;
}

/** @brief 
 *
 */
std::vector<std::tuple<double, double>> A_Star_Planner::plan(double goal_map_x, double goal_map_y) {
	
	using namespace std;

	auto[goal_grid_x, goal_grid_y] = goalToGrid(goal_map_x, goal_map_y);

	while (frontier.size() != 0) {
		expandFrontier();
	}
}

