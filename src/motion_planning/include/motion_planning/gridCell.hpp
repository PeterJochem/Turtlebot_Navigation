/** @file  
 *  @brief The gridCell class represents a cell in the occupancy grid
 *         I created class for it to hold all the data required to 
 *         implement the A Star algorithm.   
 *
 * Parameters: None
 * Publishes: None
 * Subscribes: None
 * Services: None */
#include<vector>

/** Represents a gridCell in the occupancy grid */
class gridCell {

        public:
                int x, y; // In grid coords - change the name to grid_x and grid_y
                bool explored;
                int estimatedDistance, probability;
		double cost_to_reach, est_cost_end;
		gridCell* parent;

		/** @brief Constructor for the gridCell
                 *  @param x The gridCell's x coordinate
	         *  @param y The gridCell's y coordinate
		 *  @param occupiedProbability The cell's probability of being occupied */
  		gridCell(int, int, double);
		
		/** @brief Generate all the possible children of a given grid cell
 		*          without checking if the grid cell exists or is unexplored
 		* @return A vector of tuples (x, y) that are the grid coordinates 
	 	*         of all of the current cell's children */
		std::vector<std::tuple<int, int>> generateChildren();
};
