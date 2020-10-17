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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <queue>
#include "gridCell.hpp"


// See http://neutrofoton.github.io/blog/2016/12/29/c-plus-plus-priority-queue-with-comparator/
struct gridCellCompare {
    bool operator()(const gridCell* lhs, const gridCell* rhs) {
        return lhs->estimatedDistance < rhs->estimatedDistance;
    }
};


/** @brief Implements A_Star algorithm
 */
class A_Star_Planner { 
		
	public:
		A_Star_Planner(int* map, int height, int width, int resolution);
		//A_Star_Planner(); // Default Constructor
		//void processMap(const nav_msgs::OccupancyGrid& map);	
			
		
	private:
		// Priority queue of pointers to the static grid vector
		std::priority_queue<gridCell*, std::vector<gridCell*>, gridCellCompare> frontier;
		
		// DS for searching the space
		std::vector<gridCell> grid;

		// Discretization factor?
		
		//nav_msgs::OccupancyGrid map;
		int* map; // Linear array from the nav_msgs::OccupancyGrid

		int grid_height, grid_width;
		double grid_resolution;
		int prob_threshold;


		/** @brief Describe me
		 */
		std::tuple<double, double> gridToMeters(int grid_x, int grid_y);
		
		/** @brief Describe me  
		 */
		std::tuple<int, int> metersToGrid(double map_x, double map_y);
		
		/** @brief Describe me 
		 */
		void expandFrontier();
	
		/** @brief Describe me 
		 */
		std::tuple<int, int> indexToCoords(int);
		
		/** @brief Describe me 
		 */
		int computeIndex(int, int);	

		/** @brief Describe me 
		 */
		gridCell* getGridCellPtr(int nextX, int nextY);
		
		/** @brief Describe me
		 */
		inline bool isLegal(int, int);		
		
		/** @brief Describe me
		 */
		inline bool unexplored(int, int);
		
		/** @brief Describe me 
		 */
		inline bool isFree(int, int);

		/** @brief Describe me
		 */
		std::tuple<int, int> goalToGrid(double map_x, double map_y);
		
		/** @brief Describe me 
		 */
		std::vector<std::tuple<double, double>> plan(double goal_map_x, double goal_map_y);
};
