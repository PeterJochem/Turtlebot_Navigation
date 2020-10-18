/** @file  
 *  @brief The gridCell class represents a cell in the occupancy grid
 *  	   I created class for it to hold all the data required to 
 *  	   implement the A Star algorithm.   
 *
 * Parameters: None
 * Publishes: None
 * Subscribes: None
 * Services: None */

#include "motion_planning/gridCell.hpp"
#include <tuple>
#include <vector>

/** @brief Constructor for the gridCell
 *  @param x The gridCell's x coordinate
 *  @param y The gridCell's y coordinate
 *  @param occupiedProbability The cell's probability of being occupied */
gridCell::gridCell(int x, int y, double occupiedProbability) {

        this->x = x;
        this->y = y;

        parent = nullptr;

	estimatedDistance = 1000000; // FIX ME - should be INT_MAX
	probability = occupiedProbability;
	explored = false;
}

/** @brief Generate all the possible children of a given grid cell
 *         without checking if the grid cell exists or is unexplored
 * @return A vector of tuples (x, y) that are the grid coordinates 
 *         of all of the current cell's children */
std::vector<std::tuple<int, int>> gridCell::generateChildren() {
        using namespace std;

        int leftX = x - 1;
        int leftY = y;

        int rightX = x + 1;
        int rightY = y;

        int upX = x;
        int upY = y + 1;

        int downX = x;
        int downY = y - 1;

        return {make_tuple(leftX, leftY), make_tuple(rightX, rightY), make_tuple(upX, upY), make_tuple(downX, downY) };
}
