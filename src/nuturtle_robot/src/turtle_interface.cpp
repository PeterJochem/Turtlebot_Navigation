#include "ros/ros.h"
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

using namespace rigid2d;
ros::Subscriber cmd_vel_sub;
ros::Subscriber sensor_sub;
ros::Publisher wheel_pub;
ros::Publisher joints_pub;
DiffDrive robot;

double max_motor_rot, max_cmd, encoder_ticks_rotation;
std::string left_wheel_joint, right_wheel_joint;

int initial_encoder_left = 0;
int initial_encoder_right = 0;
bool measuredInitialEncoder = false;


/* Map the desired velocities into the allowable range of values
 */
void clampVelocity(WheelVelocities& wheel_vel) {
        
	if (wheel_vel.left > max_motor_rot) {
                wheel_vel.left = max_motor_rot;
        }
        else if (wheel_vel.left < (-1 * max_motor_rot) ) {
                wheel_vel.left = -1 * max_motor_rot;
        }

        if (wheel_vel.right > max_motor_rot) {
                wheel_vel.right = max_motor_rot;
        }
        else if (wheel_vel.right < (-1 * max_motor_rot) ) {
                wheel_vel.right = -1 * max_motor_rot;
        }
}


/* When we recieve a cmd_vel, this runs and converts the 
 * desired twist into a command to the motors to set the 
 * speed of the two wheels accordingly    
 */
void cmd_vel_callback(geometry_msgs::Twist twist3D) {
		
	Twist2D twist2D = convert3DTo2D(twist3D);
	WheelVelocities wheel_vel = robot.twistToWheels(twist2D);	
	
	clampVelocity(wheel_vel);	
					
	//Convert to the nuturtlebot type {-265, 265}
	nuturtlebot::WheelCommands wheel_cmd;
	float m = (max_cmd * 2.0) / (max_motor_rot * 2.0);
  	float b = (max_cmd - max_motor_rot * m);
	
  	wheel_cmd.left_velocity = (wheel_vel.left * m) + b;
  	wheel_cmd.right_velocity = (wheel_vel.right * m) + b;		

	wheel_pub.publish(wheel_cmd);				
}

/* Convert the encoder data from the motor and convert it 
 * it to a joint states message with the position and velocity fields set 
 */
void sensor_sub_callback(nuturtlebot::SensorData newData) {
	
	double left_wheel_angle, right_wheel_angle;
	double left_wheel_velocity, right_wheel_velocity;		
	sensor_msgs::JointState state;			
	
	// Initial encoder angle depends on hardware/wheel
	// Measure all angles relative to whatever that angle is	
	if (!measuredInitialEncoder) {
		measuredInitialEncoder = true;
		initial_encoder_left = newData.left_encoder;
		initial_encoder_right = newData.right_encoder;
	}	
	

	left_wheel_angle = normalize_angle( (newData.left_encoder - initial_encoder_left) / (encoder_ticks_rotation * 2 * PI) );  
	right_wheel_angle = normalize_angle( (newData.right_encoder - initial_encoder_right) / (encoder_ticks_rotation * 2 * PI) );
	
	WheelVelocities wheel_vels = robot.updateOdometry(left_wheel_angle, right_wheel_angle);

	// Order must match!
	state.name = {left_wheel_joint, right_wheel_joint};
	state.position = {left_wheel_angle, right_wheel_angle};
	state.velocity = {wheel_vels.left, wheel_vels.right};	

	joints_pub.publish(state);
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "turtle_interface"); 
        ros::NodeHandle n;
	
	// Get values from the param server
	double wheel_base, wheel_radius, frequency, encoder_ticks_rotation;

        n.getParam("/wheel_base", wheel_base);
	n.getParam("/wheel_radius", wheel_radius);
	n.getParam("/frequency", frequency);
	n.getParam("/motor_limit", max_motor_rot);
	n.getParam("/motor_power", max_cmd);
	n.getParam("/encoder_ticks_per_revolution", encoder_ticks_rotation);
	n.getParam("/left_wheel_joint", left_wheel_joint);
	n.getParam("/right_wheel_joint", right_wheel_joint);

	// Log?	
	//ROS_INFO("/wheel_radius is %f", wheel_radius);

	// Get the current pose? Set to (0, 0, 0)? Does it even matter for this?
	robot = DiffDrive(Transform2D(), wheel_base, wheel_radius);
				
	// How big to make the queue?
	// /wheel_cmd or /turtle1/wheel_cmd?
	wheel_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1);
	joints_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
	
	cmd_vel_sub = n.subscribe("/turtle1/cmd_vel", 1, cmd_vel_callback);
        sensor_sub = n.subscribe("/sensor_data", 1, sensor_sub_callback);
	

	// What to set this to?
  	ros::Rate loop_rate(10);
	
	ros::spin();

	return 0;
}
