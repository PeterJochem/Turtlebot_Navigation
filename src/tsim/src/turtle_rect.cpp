#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
// #include <service_client.h>

#include <sstream>

/* Implements service which teleports the turtle to the lower left corner
 * of its rectangle
 */
bool traj_reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  
  // Turtle sim has a teleport service 
  // turtleX/teleport_absolute
 	
  int i = 0;	
   
  
  return true;
}


/* Add description
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "turtle_rect");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
	
  ros::service::waitForService("turtle1/teleport_absolute", 10);

  // Use this service to spawn a turtle  
  // spawn (turtlesim/Spawn) 

  // Subscribe to the turtlesim/Pose message

  // Advertise the service
  ros::ServiceServer service = n.advertiseService("traj_reset", traj_reset);  


  // Read the parameters from the server
  	// Log each one as a ROS_INFO message
  // Wait for the service to be created/available	

  int count = 0;
  while (ros::ok()) {
   
     /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
