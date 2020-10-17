#include<vector>

// Describe me
class gridCell {

        public:
                //gridCell(); // Default constructor
                gridCell(int, int);

                int x, y;
                int parent_x, parent_y;

                std::vector<std::tuple<int, int>> generateChildren();
                bool explored;
                int estimatedDistance = 0;
                int probability = 0;
};
