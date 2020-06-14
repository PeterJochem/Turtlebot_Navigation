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
class FSM_Feedback {
  
  // One state for each leg of the route	
  bool right;
  bool turn_90;
  bool up;
  bool left;
  bool down;

  public:
    FSM_Feedback(void);
    geometry_msgs::Twist checkUpdate(turtlesim::Pose);	
     
};

/* This is the constructor for the FSM
 */
FSM_Feedback::FSM_Feedback (void) {
	right = true;	
	turn_90 = false;
	up = false;
	left = false;
	down = false;
}

/* Describe this method here
 */
geometry_msgs::Twist FSM_Feedback::checkUpdate(turtlesim::Pose currentPose) {
	
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
	
	if (right == true) {
		
		if (turn_90 == true) {
			nextTwist.angular.z = rot_vel;
		
			if ( currentPose.theta > (3.14/2) ) {
				turn_90 = false;
				nextTwist.linear.x = trans_vel;
				nextTwist.angular.z = 0;
				right = false;
				up = true;
			}
		}
		else if (currentPose.x >= (starting_x + width)) {
			// set turn 90 to true
			turn_90 = true;
			//nextTwist.angular.z = rot_vel;
		}
		else if (currentPose.x < (starting_x + width)) {
			// set linear to normal speed
			nextTwist.linear.x = trans_vel;
			right = true;	
		}
	}	
	else if(up == true) {
		
		if (turn_90 == true) {
                        nextTwist.angular.z = rot_vel;

                        if (currentPose.theta < 0) {
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
			
		if (turn_90 == true) {
                        nextTwist.angular.z = rot_vel;

                        if (currentPose.theta > (-1 * 3.14/2)) {
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
                else if (currentPose.x > starting_x) {
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


/* Describe this class
 */
class FSM_Feedforward {

  // One state for each leg of the route
  bool right;
  bool turn_90;
  bool up;
  bool left;
  bool down;
  int timeQuantas;
  double frequency;	

  public:
    FSM_Feedforward(double);
    geometry_msgs::Twist checkUpdate(turtlesim::Pose);
};

/* This is the constructor for the FSM
 */
FSM_Feedforward::FSM_Feedforward (double frequencyX) {
        right = true;
        turn_90 = false;
        up = false;
        left = false;
        down = false;
	timeQuantas = 0;
	frequency = frequencyX;
}

/* Describe this method here
 * Inputs:
 * Returns: The next twist to be sent to the Turtlebot 
 */
geometry_msgs::Twist FSM_Feedforward::checkUpdate(turtlesim::Pose currentPose) {

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


	if ( (right == true) and (turn_90 ==  false) ) {
		
		// Distance = Rate * time
		double schedLinear = width / trans_vel; 
		double schedRotational = (3.14 / 2) / rot_vel;  
		
		if ( (timeQuantas * (1.0/frequency) ) >= schedLinear ) {
			
			turn_90 = true;
			timeQuantas = 0;
		}
		else {
			timeQuantas++;
			nextTwist.linear.x = trans_vel;
		}
	}	
	
	else if ( (right == true) and (turn_90 == true) ) {
		
		// Distance = Rate * time
                double schedLinear = width / trans_vel; 
                double schedRotational = (3.14 / 2) / rot_vel;
		
		if ( (timeQuantas * (1.0/frequency) ) >= schedRotational ) {
			turn_90 = false;
                	right = false;
			up = true;
			timeQuantas = 0;
		}
		else {
			nextTwist.angular.z = rot_vel;
			timeQuantas++;	
		}
	}

	if ( (up == true) and (turn_90 ==  false) ) {

                // Distance = Rate * time
                double schedLinear = height / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedLinear ) {

                        turn_90 = true;
                        timeQuantas = 0;
                }
                else {
                        timeQuantas++;
                        nextTwist.linear.x = trans_vel;
                }
        }

        else if ( (up == true) and (turn_90 == true) ) {

                // Distance = Rate * time
                double schedLinear = height / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedRotational ) {
                        turn_90 = false;
                        up = false;
                        left = true;
                        timeQuantas = 0;
                }
                else {
                        nextTwist.angular.z = rot_vel;
                        timeQuantas++;
                }
        }

	if ( (left == true) and (turn_90 ==  false) ) {

                // Distance = Rate * time
                double schedLinear = width / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedLinear ) {

                        turn_90 = true;
                        timeQuantas = 0;
                }
                else {
                        timeQuantas++;
                        nextTwist.linear.x = trans_vel;
                }
        }

        else if ( (left == true) and (turn_90 == true) ) {

                // Distance = Rate * time
                double schedLinear = width / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedRotational ) {
                        turn_90 = false;
                        left = false;
                        down = true;
                        timeQuantas = 0;
                }
                else {
                        nextTwist.angular.z = rot_vel;
                        timeQuantas++;
                }
        }

	if ( (down == true) and (turn_90 ==  false) ) {

                // Distance = Rate * time
                double schedLinear = height / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedLinear ) {

                        turn_90 = true;
                        timeQuantas = 0;
                }
                else {
                        timeQuantas++;
                        nextTwist.linear.x = trans_vel;
                }
        }

        else if ( (down == true) and (turn_90 == true) ) {

                // Distance = Rate * time
                double schedLinear = height / trans_vel;
                double schedRotational = (3.14 / 2) / rot_vel;

                if ( (timeQuantas * (1.0/frequency) ) >= schedRotational ) {
                        turn_90 = false;
                        left = false;
                        down = true;
                        timeQuantas = 0;
                }
                else {
                        nextTwist.angular.z = rot_vel;
                        timeQuantas++;
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
  	
	currentPose = *pose;

	return;
}

/* describe this function
 */
void logParams(void) {
	
	double height;
        double width;
        double rot_vel;
        double trans_vel;
        double starting_x;
        double starting_y;
	
	ros::NodeHandle n;
        ROS_INFO("/height is %d", n.getParam("/height", height) );
        
	ROS_INFO("/width is %d", n.getParam("/width", width) ); 
	ROS_INFO("/rot_vel is %d", n.getParam("/rot_vel", rot_vel) );
        ROS_INFO("/trans_vel is %d", n.getParam("/trans_vel", trans_vel) );
	ROS_INFO("/x is %d", n.getParam("/x", starting_x) );
        ROS_INFO("/y is %d", n.getParam("/y", starting_y) );
	
}

/* Add description
 */
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "turtle_rect");

  ros::NodeHandle n;
  
  logParams();
   
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
  //FSM_Feedback myFSM = FSM_Feedback();
  FSM_Feedforward myFSM = FSM_Feedforward( double(frequency) );

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
