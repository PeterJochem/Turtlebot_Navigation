#include "ros/ros.h"
#include <gtest/gtest.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

ros::Publisher cmd_vel_pub;
ros::Subscriber wheel_cmd_sub;

ros::Publisher sensor_pub;
ros::Subscriber joint_states_sub;

nuturtlebot::WheelCommands observed_wheel_cmd;
sensor_msgs::JointState observed_joint_state;

bool waitForTurtleInterface = false;
int ticks_per_rotation;

/* Describe 
 */
void wheel_cmd_callback(nuturtlebot::WheelCommands wheel_cmd) {
	
	observed_wheel_cmd = wheel_cmd;
	waitForTurtleInterface = false;
}

/* Describe 
 */
void joint_states_callback(sensor_msgs::JointState joint_state) {

        observed_joint_state = joint_state;
        waitForTurtleInterface = false;
}


/*
TEST(TestSuite, cmd_vel_translate_only) {
	
	geometry_msgs::Twist translateOnly;
	translateOnly.linear.x = 1.0;
	
	waitForTurtleInterface = true;
	cmd_vel_pub.publish(translateOnly);
	
	//ros::spinOnce();	
	while (waitForTurtleInterface) {
		ros::spinOnce();
	}
	
	waitForTurtleInterface = false;
	ASSERT_NEAR(observed_wheel_cmd.left_velocity, observed_wheel_cmd.right_velocity, 1e-3);	
}
*/

/*
TEST(TestSuite, cmd_vel_rotate_only) {
	
	geometry_msgs::Twist rotateOnly;
        rotateOnly.angular.x = 0.0;
        rotateOnly.angular.y = 0.0;
        rotateOnly.angular.z = 1.0;

	waitForTurtleInterface = true;
        cmd_vel_pub.publish(rotateOnly);

        //ros::spinOnce();      
        while (waitForTurtleInterface) {
                ros::spinOnce();
        }

        waitForTurtleInterface = false;
        ASSERT_NEAR(observed_wheel_cmd.left_velocity, -1 * observed_wheel_cmd.right_velocity, 1e-3);
}
*/

TEST(TestSuite, cmd_vel_rotate_and_translate) {
}

/* Verifies that encoder data on sensors
 * is converted to joint_states properly
 */
TEST(TestSuite, encoders) {
	
	using namespace rigid2d;
	// Publish fake encoder data that indicates we rotated exactly 1/4 rotation
	// Verify that the angle is +/-PI/2
	nuturtlebot::SensorData sensor_data;
	sensor_data.left_encoder = ticks_per_rotation / 4;
	sensor_data.right_encoder = ticks_per_rotation / 4;

	//waitForTurtleInterface = true;
        waitForTurtleInterface = false;
	sensor_pub.publish(sensor_data);

        ros::spinOnce();      
        while (waitForTurtleInterface) {
                ros::spinOnce();
        }

        waitForTurtleInterface = false;
        ASSERT_NEAR(observed_joint_state.position[0], PI/2, 1e-3);
	ASSERT_NEAR(observed_joint_state.position[1], PI/2, 1e-3);
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_turtle_interface_node");
  
  ros::NodeHandle n;
  //n.getParam("/encoder_ticks_per_revolution", ticks_per_rotation);
  ticks_per_rotation = 4096;

  // true makes it a latched publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1, true);    
  sensor_pub = n.advertise<nuturtlebot::SensorData>("/sensor_data", 1, true);

  wheel_cmd_sub = n.subscribe("/wheel_cmd", 1, wheel_cmd_callback); 	
  joint_states_sub = n.subscribe("/joint_states", 1, joint_states_callback); 
	

  return RUN_ALL_TESTS();
}
