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
	
	geometry_msgs::Twist nextTwist;
	nextTwist.linear.x = 0.0;
  	nextTwist.linear.y = 0.0;
  	nextTwist.linear.z = 0.0;

  	nextTwist.angular.x = 0.0;
  	nextTwist.angular.y = 0.0;
  	nextTwist.angular.z = 0.0;

	if ( right == true ) {
		
		if (turn_90 == true) {
			nextTwist.angular.z = 0.1;
		}

		else if (currentPose.x > 4) {
			// set turn 90 to true
			turn_90 = true;
			nextTwist.angular.z = 0.1;

		}
		else if (currentPose.x < 4) {
			// set linear to normal speed
			nextTwist.linear.x = 1.0;
		}


	}	
	else if(up == true) {
		// 
	}
	else if (left == true) {
		//
	}
	else if (down == true) {
		//
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
