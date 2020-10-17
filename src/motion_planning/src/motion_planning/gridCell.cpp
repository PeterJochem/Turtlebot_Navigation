// Add a file header

#include <vector>
#include "motion_planning/gridCell.hpp"
#include <tuple>

/** @brief Describe me
 */
gridCell::gridCell(int x, int y) {

        this->x = x;
        this->y = y;

        parent_x = 0;
        parent_y = 0;
}


/** @brief Generate all the possible children of a given grid cell
 * without checking if the grid cell exists or is unexplored
 */
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



