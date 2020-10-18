/** @file
 *  @brief This file implements an A Star planning algorithm. It 
 *         is meant to be used with ROS nav_msgs::Occupancy grids 
 *         but is flexible enough to work with any sort of array
 *         which represents occupancy probabilities  
 *
 * Parameters: None 
 * Publishes: None
 * Subscribes: None
 * Services: None  */
#include "gridCell.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include <queue>

// Custom comparator for the priority_queue of gridCell pointers
// See http://neutrofoton.github.io/blog/2016/12/29/c-plus-plus-priority-queue-with-comparator/
struct gridCellCompare {
    bool operator()(const gridCell* lhs, const gridCell* rhs) {
        return lhs->estimatedDistance < rhs->estimatedDistance;
    }
};

/** @brief Implements A_Star algorithm */
class A_Star_Planner { 
		
	public:
		/** @brief Constructor
		 *  @param map linear array (row major order) of occupancy probabilities. 
		 *  	   [-1, 100] where -1 means unknown
		 *  @param height Height of the map in grid units
		 *  @param width Width of the map in grid units 
		 *  @param resolution Describes the relative scale between the map and the grid (m/cell) */
		A_Star_Planner(int* map, int height, int width, int resolution);
		
		//void processMap(const nav_msgs::OccupancyGrid& map);	
			
		/** @brief Plan a path and return it as a vector of (x, y) tuples
		 *  @return vector of (x, y) tuples */
                std::vector<std::tuple<double, double>> plan();	

		/** @brief Check if the starting and finishing positions of the path are 
		 * 	   defined in the map frame and update fields with values if legal.
		 *  @param start_map_x Path's starting x coordinate (meters) in the map frame 
                 *  @param start_map_y Path's starting y coordinate (meters) in the map frame
		 *  @param goal_map_x Path's destination x coordinate (meters) in the map frame
		 *  @param goal_map_y Path's destination y coordinate (meters) in the map frame 
		 *  @return True if the start and goal points are legal, False otherwise */
                bool setGoal(double start_map_x, double start_map_y, double goal_map_x, double goal_map_y);
				
	private:
		// Priority queue of pointers to the static grid vector below
		std::priority_queue<gridCell*, std::vector<gridCell*>, gridCellCompare> frontier;
		std::vector<gridCell> grid; // Linear vector represents the map in row major order

		//nav_msgs::OccupancyGrid map;
		int* map; // Linear array of occupancy probabilities (from nav_msgs::OccupancyGrid)

		int grid_height, grid_width, goal_grid_x, goal_grid_y;
		int prob_threshold; // Arbitary probability where we will consider paths through
		
		double goal_map_x, goal_map_y, start_map_x, start_map_y, grid_resolution;
		bool goalSet, goalReached;

		/** @brief Convert grid tuple (x, y) to the same point in the map frame (meters)
		 *  @param grid_x The x coordinate in the map frame
		 *  @param grid_y The y coordinate in the map frame 
		 *  @return Tuple (x, y) of the point in the map frame */
		std::tuple<double, double> gridToMeters(int grid_x, int grid_y);
		
		/** @brief Convert a point from the map frame (meters) to the grid coordinates
		 *  @param map_x The x coordinate in the map frame   
		 *  @param map_y The y coordinate in the map frame
		 *  @return Tuple (x, y) of the point in the grid coordinate system */
		std::tuple<int, int> metersToGrid(double map_x, double map_y);
		
		/** @brief Pop top of frontier off list and add its children to the frontier  
		 *  @return True if we pop off the goal node. False otherwise */
		bool expandFrontier();
	
		/** @brief Convert an index of the vector of gridCells into the gridCells (x, y) location  
		 *  @param Index is the gridCell's index in the linear array
		 *  @return Tuple of the grid's location (x, y) in grid coordinates */
		std::tuple<int, int> indexToCoords(int);
		
		/** @brief Given the goal coordinates in grid cells, compute the index
                 *  	   of the cell item in the linear data array from the OccupancyGrid 
                 *         The occupancy grid is in row-major order
                 *  @param grid_x The gridCell's x coordinate in the grid
 		 *  @param grid_y The gridCell's y coordinate in the grid
		 *  @return The index in the map array of the gridCell at location (grid_x, grid_y) */
		int computeIndex(int x, int y);	

		/** @brief Compute and return pointer to the gridCell object 
 		*          at the grid coordinate (grid_x, grid_y)
 		*  @param grid_x The gridCell x coordinate 
 		*  @param grid_y The gridCell y coordinate 
 		*  @return A gridCell pointer to the give gridCell at (grid_x, grid_y) */
		gridCell* getGridCellPtr(int nextX, int nextY);
		
		/** @brief Determine if the (x, y) pair in grid coordinates 
 		*          actually exists on the map 
 		*  @return True if the coordinates exist in the grid, false otherwise */	
		inline bool isLegal(int, int);		
		
		/** @brief Check if the (x, y) pair in grid coordinates
 		*          can be added to the frontier. Have we already added it
 		*          to the frontier? 
 		*  @return True if the grid cell is unexplored. False otherwise */	
		inline bool unexplored(int, int);
		
		/** @brief Check if the gridCell coordinates are occupied or not.
	 	*          Uses the arbitary field prob_threshold as the cutoff for
 		*          determining if the cell is occupied
 		*  @param grid_x The gridCell's x coordinate
 		*  @param grid_y The gridCell's y coordinate
 		*  @return True if the gridCell is not occupied */
		inline bool isFree(int, int);

		/** @brief Given the goal in the map frame, find the closest grid cell coordinates
 		*   @param map_x The coordinate's x position
 		*   @param map_y The coordinate's y position
 		*   @return A tuple of the (x, y) location in the grid system */		
		std::tuple<int, int> goalToGrid(double map_x, double map_y);
		
		/** @brief Check if a given pair of coordinates is the goal location
 		*   @param grid_x The gridCell's x coordinate
 		*   @param grid_y The gridCell's y coordinate
 		*   @return True if the pair of coordinates is the goal, false otherwise */	
		inline bool isGoal(int grid_x, int grid_y);

		/** @brief Once we find the goal node, this backtracks through 
 		*          the parent pointer to record the optimal path
 		*   @return A vector of (x, y) tuples (in the grid system) that is the path */	
		std::vector<std::tuple<double, double>> backtrack();
		
		/** @brief Check if a given point (in the map frame) is even included in the 
 		*          given occupancy grid
 		*  @param goal_map_x The proposed goal point's (in the map frame) x coordinate
 		*  @param goal_map_y The proposed goal point's (in the map frame) y coordinate
 		*  @return True if the point is legal, false otherwise */		
		bool isGoalPointLegal(double, double);
		
		/** @brief Check if a given point (in the map frame) is even included in the 
 		*          given occupancy grid
		*  @param goal_map_x The proposed starting point's (in the map frame) x coordinate
		*  @param goal_map_y The proposed starting point's (in the map frame) y coordinate
 		*  @return True if the point is legal, false otherwise */	
		bool isStartPointLegal(double, double);
};	
