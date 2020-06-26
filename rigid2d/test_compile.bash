# Implement
g++ -Wall -Wextra -g -std=c++17 -I /usr/include/eigen3 -o rigid2d_test main.cpp rigid2d.cpp -ljsoncpp

./rigid2d_test 0

./rigid2d_test 1

./rigid2d_test 2

./rigid2d_test 3

./rigid2d_test 4

./rigid2d_test 5
