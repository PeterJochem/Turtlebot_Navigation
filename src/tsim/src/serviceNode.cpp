#include "ros/ros.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <tsim/PoseError.h>
#include <tsim/traj_reset.h>
#include <cstdlib>

ros::ServiceClient teleport_client;
ros::ServiceClient reset_client;



/* Implements service which teleports the turtle to the lower left corner
 * of its rectangle
 */
bool traj_reset(tsim::traj_reset::Request &req, tsim::traj_reset::Response &res) {

  // Turtle sim has a teleport service 
  // turtleX/teleport_absolute

  turtlesim::TeleportAbsolute tele_srv;
  tele_srv.request.x = 1.0;
  tele_srv.request.y = 1.0;
  tele_srv.request.theta = 0.0;

  ROS_INFO("traj_reset service called");
  return teleport_client.call(tele_srv);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "client");
  
  ros::NodeHandle n;
  
  ros::service::waitForService("turtle1/teleport_absolute", -1);
  teleport_client = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

  // Advertise the service
  ros::ServiceServer service = n.advertiseService("/tsim/traj_reset", traj_reset);
  
  // This specifies the rate at which we loop - this means loop at 1000 Hz
  ros::Rate loop_rate(1000);

  while (ros::ok()) {

	ros::spinOnce();

	loop_rate.sleep();
  } 

  return 0;
}
