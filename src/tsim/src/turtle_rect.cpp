#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
// #include <my_custom_msg_package/MyCustomMsg.h>
#include <tsim/PoseError.h>
#include <tsim/traj_reset.h>
#include <sstream>
#include <stdlib.h>

ros::ServiceClient teleport_client;
ros::ServiceClient setPen_client;
ros::ServiceClient reset_client;
turtlesim::Pose currentPose;

/* Describe this class 
 */
class FSM {
  
  // One state for each leg of the route	
  bool right;
  bool turn_90;
  bool up;
  bool left;
  bool down;

  public:
    void set_values (int,int);
    FSM(void);
    geometry_msgs::Twist checkUpdate(turtlesim::Pose);	
     
};

/* This is the constructor for the FSM
 */
FSM::FSM (void) {
	right = true;	
	turn_90 = false;
	up = false;
	left = false;
	down = false;
}

/* Describe this method here
 */
geometry_msgs::Twist FSM::checkUpdate(turtlesim::Pose currentPose) {
	
	ros::NodeHandle n;
 	
	// Read the parameters from the server	
	std::string s;		
	
	double height; 
	double width;
	double rot_vel;
	double trans_vel;
	double starting_x;
	double starting_y;

	n.getParam("/height", height);
	n.getParam("/width", width);
	n.getParam("/rot_vel", rot_vel);
	n.getParam("/trans_vel", trans_vel);	
	n.getParam("/x", starting_x);
	n.getParam("/y", starting_y);


	geometry_msgs::Twist nextTwist;
	nextTwist.linear.x = 0.0;
  	nextTwist.linear.y = 0.0;
  	nextTwist.linear.z = 0.0;

  	nextTwist.angular.x = 0.0;
  	nextTwist.angular.y = 0.0;
  	nextTwist.angular.z = 0.0;
	
	std::cout << "function called " << right << std::endl;

	if ( right == true ) {
		
		// std::cout << "function called and turn_90 is " << turn_90 << std::endl;	

		if (turn_90 == true) {
			nextTwist.angular.z = rot_vel;
		
			// std::cout << "case 1a\n";
			if ( currentPose.theta > (3.14/2) ) {
				turn_90 = false;
				nextTwist.linear.x = trans_vel;
				nextTwist.angular.z = 0;
				right = false;
				up = true;
				// std::cout << "case 1b\n";
			}
		}
		else if (currentPose.x >= (starting_x + width)) {
			// set turn 90 to true
			turn_90 = true;
			//nextTwist.angular.z = rot_vel;
			// std::cout << "case 2\n";
		}
		else if (currentPose.x < (starting_x + width)) {
			// set linear to normal speed
			nextTwist.linear.x = trans_vel;
			right = true;	
			// std::cout << "case 3\n";
		}
	}	
	else if(up == true) {
		
		// std::cout << "moving up " << std::endl;
		if (turn_90 == true) {
                        nextTwist.angular.z = rot_vel;

                        if ( currentPose.theta < 0 ) {
				// std::cout << "angle less than 0" << std::endl;
                                turn_90 = false;
                                nextTwist.linear.x = trans_vel;
				nextTwist.angular.z = 0;
				up = false;
				left = true;
                        }
                }
                else if (currentPose.y >= (starting_y + height)) {
                        // set turn 90 to true
                        turn_90 = true;
                        nextTwist.angular.z = rot_vel;

                }
                else if (currentPose.y < (starting_y + height)) {
                        // set linear to normal speed
                        nextTwist.linear.x = trans_vel;
                } 
	}
	else if (left == true) {
		
		// std::cout << "left is true" << std::endl;
			
		if (turn_90 == true) {
                        nextTwist.angular.z = rot_vel;

                        if ( currentPose.theta > (-1 * 3.14/2) ) {
                                turn_90 = false;
                                nextTwist.linear.x = trans_vel;
				nextTwist.angular.z = 0;
                                left = false;
                                down = true;
                        }
                }
                else if (currentPose.x <= starting_x) {
                        // set turn 90 to true
                        turn_90 = true;
                        //nextTwist.angular.z = rot_vel;
			

                }
                else if (currentPose.x > starting_x ) {
                        // set linear to normal speed
                        nextTwist.linear.x = trans_vel;
                }
	}
	else if (down == true) {

                if (turn_90 == true) {
                        nextTwist.angular.z = rot_vel;

                        if ( currentPose.theta > 0 ) {
                                turn_90 = false;
                                nextTwist.linear.x = trans_vel;
				nextTwist.angular.z = 0;
                                down = false;
                                right = true;
                        }
                }
                else if (currentPose.y <= starting_y) {
                        // set turn 90 to true
                        turn_90 = true;
                        nextTwist.angular.z = rot_vel;
                }
                else if (currentPose.y > starting_y ) {
                        // set linear to normal speed
                        nextTwist.linear.x = trans_vel;
                }
	}
	
	return nextTwist;
}

/* Describe this method 
 */
void makeSetPen(bool on, turtlesim::SetPen& sp) {
	
   unsigned int a = 1;
   sp.request.r = a;
   sp.request.g = a;
   sp.request.b = a;
   sp.request.width = a;
   sp.request.off = !on;
	
   return;
}

/* This is the callback for the subscriber to the pose messages
 * of turtle1
 */
void poseCallback(const turtlesim::Pose::ConstPtr& pose) {
  	
	// ROS_INFO("Posecallback called");	
	currentPose = *pose;

	return;
}

/* Add description
 */
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "turtle_rect");

  ros::NodeHandle n;
  
  // Advertise the service
  // ros::ServiceServer service = n.advertiseService("/tsim/traj_reset", traj_reset);
  
  // Read the parameters from the server
        // Log each one as a ROS_INFO message
  // Wait for the service to be created/available
   
  ros::service::waitForService("turtle1/teleport_absolute", -1); 
  teleport_client = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");            
   
    
  // Lift the pen
  ros::service::waitForService("turtle1/set_pen", -1);
  setPen_client = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  turtlesim::SetPen sp;
  makeSetPen(false, sp);
  setPen_client.call(sp);
  
  // Teleport the turtle to the start
  ros::service::waitForService("tsim/traj_reset", -1);
  ros::ServiceClient reset_client = n.serviceClient<tsim::traj_reset>("tsim/traj_reset");
  tsim::traj_reset srv;
  reset_client.call(srv);
  //
  /*
  turtlesim::TeleportAbsolute tele_srv;
  tele_srv.request.x = 5;
  tele_srv.request.y = 5;
  tele_srv.request.theta = 0.0;
 
  ROS_INFO("traj_reset service called");
  bool run = teleport_client.call(tele_srv);
  */
  ros::spinOnce();
  // Put pen down
  makeSetPen(true, sp);
  setPen_client.call(sp);
  
  // How big to make this queue 
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  
  // Publishes the error between actual pose and the desired pose
  ros::Publisher error_pose_pub = n.advertise<tsim::PoseError>("turtle1/pose_error", 1000);

  // Subscribe to the turtle's pose
  ros::Subscriber sub = n.subscribe("/turtle1/pose", 1, poseCallback);

  int frequency = 1000;
  // This specifies the rate at which we loop - this means loop at 1000 Hz
  ros::Rate loop_rate(frequency);
 
  // Create our FSM to record state
  FSM myFSM = FSM();

  turtlesim::Pose poseNow;
  int count = 0;
 	 
  while (ros::ok()) {
	
	  if (count == 0) {
		ros::service::waitForService("tsim/traj_reset", -1);	
		ros::ServiceClient reset_client = n.serviceClient<tsim::traj_reset>("tsim/traj_reset");
	  	tsim::traj_reset srv;
  		// reset_client.call(srv);
	}	

    // Get the turtle's current position
    // This is done by our subscription - it fills global variable
    // Copy the pose to a new variable to avoid asynchronous errors
    poseNow = currentPose;
	
    geometry_msgs::Twist currentTwist = myFSM.checkUpdate(poseNow); 

    // Compute the error between actual position and position it would 
    // be in if it had followed the command exactly
    tsim::PoseError current_error;
    current_error.x_error = ( (1 / float(frequency)) * currentTwist.linear.x) - poseNow.x;
    current_error.y_error = ( (1 / float(frequency)) * currentTwist.linear.y) - poseNow.y;
    current_error.theta_error = ( (1 / float(frequency)) * currentTwist.angular.z) - poseNow.theta; 
    
    // Publish the current error 
    error_pose_pub.publish(current_error);
     
      
    // Issue a new twist 
    cmd_vel_pub.publish(currentTwist);

    // Do I need this?
    // Yes, this is when callbacks get called
    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
