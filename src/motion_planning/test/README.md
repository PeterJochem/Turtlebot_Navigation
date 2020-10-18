# Testing the Motion Planning Package
I made the motion planning class general enough that I could test its functionality independently of ROS. For example, there is no need to generate simulated laser data in Gazebo in order to test most of the planner's functionality. 

# test_astar.cpp
This is the file that defines the tests to be run via gtest

# test_grids folder
This has csv files that represent very small occupancy grids. These are read by test_astar.cpp and used to test some of the motion planner's functionality. 
