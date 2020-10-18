/** @file  
 *  @brief This file implements an A Star planning algorithm. It 
 *         is meant to be used with ROS nav_msgs::Occupancy grids 
 *         but is flexible enough to work with any sort of array
 *         which represents occupancy probabilities 
 *
 * Parameters: None
 * Publishes: None
 * Subscribes: None
 * Services: None */
#include "motion_planning/A_Star_Planner.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include "ros/ros.h"

/** @brief Determine if the (x, y) pair in grid coordinates 
 * 	   actually exists on the map 
 *  @return True if the coordinates exist in the grid, false otherwise */
inline bool A_Star_Planner::isLegal(int grid_x, int grid_y) {
	
	if (grid_x < 0 || grid_x >= grid_width || grid_y < 0 || grid_y >= grid_height) {
		return false;
	}
	return true;
}

/** @brief Check if the (x, y) pair in grid coordinates
 *         can be added to the frontier. Have we already added it
 *         to the frontier? 
 *  @return True if the grid cell is unexplored. False otherwise */
inline bool A_Star_Planner::unexplored(int grid_x, int grid_y) {
	
	int index = computeIndex(grid_x, grid_y);
	return !(grid[index].explored);
}

/** @brief Check if the gridCell coordinates are occupied or not.
 * 	   Uses the arbitary field prob_threshold as the cutoff for
 * 	   determining if the cell is occupied
 *  @param grid_x The gridCell's x coordinate
 *  @param grid_y The gridCell's y coordinate
 *  @return True if the gridCell is not occupied */
inline bool A_Star_Planner::isFree(int grid_x, int grid_y) {
	
	int index = computeIndex(grid_x, grid_y);
        return grid[index].probability < prob_threshold;
}

/** @brief Check if a given pair of coordinates is the goal location
 *  @param grid_x The gridCell's x coordinate
 *  @param grid_y The gridCell's y coordinate
 *  @return True if the pair of coordinates is the goal, false otherwise */
inline bool A_Star_Planner::isGoal(int grid_x, int grid_y) {
	
	if (grid_x == goal_grid_x && grid_y == goal_grid_y) {
		return true;
	}	
	return false;
}

/** @brief Pop the topmost element from the frontier and add its children
 *         to the frontier.  
 *  @return True if we pop the goal node of the frontier, false otherwise */
bool A_Star_Planner::expandFrontier() {
	using namespace std;

	if (frontier.size() == 0) {
		return false;
	}

	gridCell* currentCell = frontier.top();
	frontier.pop(); 
	
	if (isGoal(currentCell->x, currentCell->y)) {
		std::cout << "Found solution" << std::endl;
		return true;
	}	

	vector<tuple<int, int>> allChildrenCoords = currentCell->generateChildren();

	for (unsigned int i = 0; i < allChildrenCoords.size(); i++) {
		
		auto[nextX, nextY] = allChildrenCoords.at(i); 
		if (isLegal(nextX, nextY) && unexplored(nextX, nextY) && isFree(nextX, nextY)) {
			
			// add child to the frontier
			gridCell* nextCell = getGridCellPtr(nextX, nextY);
			nextCell->explored = true;
			nextCell->parent = currentCell;		
			frontier.push(nextCell);
		}	
	}

	return false;
}


/** @brief Constructor for the A_Star Planner
 *  @param map Linear array of 
 *  @param grid_height The height of the grid
 *  @param grid_width The width of the grid
 *  @param grid_resolution Determines ratio between grid units and map units - ratio of meters/cell */
A_Star_Planner::A_Star_Planner(int* map, int grid_height, int grid_width, int grid_resolution) {
	using namespace std;

	this->map = map;
	frontier = priority_queue<gridCell*, std::vector<gridCell*>, gridCellCompare>();

	this->grid_height = grid_height;
	this->grid_width = grid_width;
	this->grid_resolution = grid_resolution;
	prob_threshold = 50; // Make this a parameter on the ROS server

	// Remember to delete this
	grid = vector<gridCell>();
	grid.reserve(grid_width * grid_height);	
	
	for (int i = 0; i < grid_width * grid_height; i++) {
		
		auto[nextX, nextY] = indexToCoords(i);	
		grid.push_back(gridCell(nextX, nextY, map[i]));
	}
}

/** @brief Convert coordinates from the grid frame to the map frame 
 *  @param grid_x The gridCell x coordinate 
 *  @param grid_y The gridcell y coordinate
 *  @return A tuple of the (x, y) point in the map frame (meters) */
std::tuple<double, double> A_Star_Planner::gridToMeters(int grid_x, int grid_y) {

	double x_map = grid_x / grid_resolution; 			
	double y_map = grid_y / grid_resolution;
		
	return {x_map, y_map};
}

/** @brief Convert the linear array index into the grid system coordinates
 *  @param index The index to the linear array of occupancy probabilities
 *  @return A tuple of where the gridCell is in the grid frame */
std::tuple<int, int> A_Star_Planner::indexToCoords(int index) {
	
	int nextX = index % grid_width; 
	int nextY = index / grid_width;
	return {nextX, nextY};
}

/** @brief Compute and return pointer to the gridCell object 
 *         at the grid coordinate (grid_x, grid_y)
 *  @param grid_x The gridCell x coordinate 
 *  @param grid_y The gridCell y coordinate 
 *  @return A gridCell pointer to the give gridCell at (grid_x, grid_y) */
gridCell* A_Star_Planner::getGridCellPtr(int grid_x, int grid_y) {
	
	int index = computeIndex(grid_x, grid_y);
	return &(grid[index]);
}

/** @brief Convert a point in the map frame into the grid system   
 *  @param map_x The coordinate's x position
 *  @param map_y The coordinate's y position
 *  @return A tuple of the (x, y) location in the grid system */
std::tuple<int, int> A_Star_Planner::metersToGrid(double map_x, double map_y) {
	
        int x_grid = map_x * grid_resolution; // Truncates the fractional part
        int y_grid = map_y * grid_resolution;
        return {x_grid, y_grid};
}

/** @brief Given the goal in the map frame, find the closest grid cell coordinates
 *  @param map_x The coordinate's x position
 *  @param map_y The coordinate's y position
 *  @return A tuple of the (x, y) location in the grid system */
std::tuple<int, int> A_Star_Planner::goalToGrid(double map_x, double map_y) {

	auto[x_grid, y_grid] = metersToGrid(map_x, map_y);
	
	// Check for -grid indexes
        if (x_grid >= grid_width || y_grid >= grid_height) {
                // We are out of bounds
        }

        return {x_grid, y_grid};
}

/** @brief Given the goal coordinates in grid cells, compute the index
 *         of the cell item in the linear data array from the OccupancyGrid 
 *         The occupancy grid is in row-major order
 *  @param grid_x The gridCell's x coordinate in the grid
 *  @param grid_y The gridCell's y coordinate in the grid
 *  @return The index in the map array of the gridCell at location (grid_x, grid_y) */
int A_Star_Planner::computeIndex(int grid_x, int grid_y) {
	
	return (grid_y * grid_width) + grid_x;
}

/** @brief Once we find the goal node, this backtracks through 
 * 	   the parent pointer to record the optimal path
 *  @return A vector of (x, y) tuples (in the grid system) that is the path */
std::vector<std::tuple<double, double>> A_Star_Planner::backtrack() {
	using namespace std;

	gridCell* currentCell = getGridCellPtr(goal_grid_x, goal_grid_y);
	vector<tuple<double, double>> path = vector<tuple<double, double>>();

	while (currentCell->parent != nullptr) {
				
		path.push_back(make_tuple(currentCell->x, currentCell->y));
		currentCell = currentCell->parent;
	}

	path.push_back(make_tuple(currentCell->x, currentCell->y));
	std::reverse(path.begin(), path.end());
	return path;	
}


/** @brief Plans a path from the starting grid cell to the goal grid cell
 *  @return A vector of (x, y) tuples (in the grid frame) that are the path */
std::vector<std::tuple<double, double>> A_Star_Planner::plan() {
	using namespace std;

	if (!goalSet) {
		cout << "No goal set. Returning null";
		return std::vector<std::tuple<double, double>>();
	}
		
	auto[start_grid_x, start_grid_y] = metersToGrid(start_map_x, start_map_y);	
	
	getGridCellPtr(start_grid_x, start_grid_y)->explored = true;
        frontier.push(getGridCellPtr(start_grid_x, start_grid_y));

	while (frontier.size() != 0) {
		if (expandFrontier()) {
			// We found a solution
			
			std::vector<std::tuple<double, double>> myPts = backtrack();
			for (int i = 0; i < myPts.size(); i++) {
				
				auto[x, y] = myPts[i];
				std::cout << "(" <<  x << ", " << y << ")" << " -> "; 
			}
			cout << endl;
		}
	}

	// FIX ME - return a real list of points
	return std::vector<std::tuple<double, double>>();
}

/** @brief Check if a given point (in the map frame) is even included in the 
 *  	   given occupancy grid
 *  @param goal_map_x The proposed goal point's (in the map frame) x coordinate
 *  @param goal_map_y The proposed goal point's (in the map frame) y coordinate
 *  @return True if the point is legal, false otherwise */
bool A_Star_Planner::isGoalPointLegal(double goal_map_x, double goal_map_y) {	
	return true;
}

/** @brief Check if a given point (in the map frame) is even included in the 
 *         given occupancy grid
 *  @param goal_map_x The proposed starting point's (in the map frame) x coordinate
 *  @param goal_map_y The proposed starting point's (in the map frame) y coordinate
 *  @return True if the point is legal, false otherwise */
bool A_Star_Planner::isStartPointLegal(double goal_start_x, double goal_start_y) {
        return true;
}

/** @brief Set the planner's start and end points
 *  @param start_map_x The proposed starting point's (in the map frame) x coordinate
 *  @param start_map_y The proposed starting point's (in the map frame) y coordinate
 *  @param goal_map_x The proposed goal point's (in the map frame) x coordinate
 *  @param goal_map_y The proposed goal point's (in the map frame) y coordinate
 *  @return True if the proposed starting and end points are legal, false otherwise */
bool A_Star_Planner::setGoal(double start_map_x, double start_map_y, double goal_map_x, double goal_map_y) {
	
	this->goal_map_x = goal_map_x;
	this->goal_map_y = goal_map_y;
		
	this->start_map_x = start_map_x;
	this->start_map_y = start_map_y;

	std::tie(goal_grid_x, goal_grid_y) = goalToGrid(goal_map_x, goal_map_y);    
	
	// Make sure the goal is legal
	if (!isStartPointLegal(start_map_x, start_map_y) || !isGoalPointLegal(goal_map_x, goal_map_y)) {
		return false;
	}
	
	goalSet = true;
	return false;
}
