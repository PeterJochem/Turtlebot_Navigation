/** @brief  
 */
#include "motion_planning/A_Star_Planner.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include "ros/ros.h"

void processMap(const nav_msgs::OccupancyGrid& map) {

        // std::cout << std::endl << (int)map.data[0] << std::endl;     




        return;
}

int main(int argc, char **argv) {

        ros::init(argc, argv, "motion_planning_node");
        ros::NodeHandle n;
	
	/*
        n.getParam("/rotational_vel_limit", max_rotation_speed);
        n.getParam("/trans_vel_limit", max_translational_speed);
        n.getParam("/k_p_trans", k_p_trans);
        n.getParam("/k_i_trans", k_i_trans);
        n.getParam("/k_p_rot", k_p_rot);
        n.getParam("/k_i_rot", k_i_rot);
        n.getParam("/linear_threshold", linear_threshold);
        n.getParam("/angular_threshold", angular_threshold);
	*/
		
        //A_Star_Planner(int* map, int height, int width, int resolution);
        A_Star_Planner myPlanner = A_Star_Planner(nullptr, 1, 1, 100);
        ros::Subscriber map_sub = n.subscribe("/map", 1, processMap);


        while (ros::ok()) {
                ros::spinOnce();
                // Check if there are new map messages, if so, refresh data/look to refresh data
        }


        return 0;
}

