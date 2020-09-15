# Description
This ROS package is used for simulating the robot in Gazebo. I wrote a plugin for Gazebo to simulate my differential drive robot. This allows me to iterate faster and avoid using the robot. The plugin listens for wheel commands that are normally sent to the real robot's motors and uses them to update the simulated robot's wheel velocities. The plugin also simulates the encoder and laser data normally published by the real robot. This allows us to have a pretty realistic and dynamic simulation of the robot and its enviroment.

# Run code 
To launch Gazebo and have the Turtlebot navigate to a series of waypoints, run ```roslaunch nuturtle_gazebo diff_drive_gazebo.launch``` <br />

# Results
Below is a video of the robot navigating to waypoints in gazebo using a feedback controller. It also shows RVIZ which displays the robot's internal odometry data. This is where the robot thinks it is. The coordinate axes are not aligned but you can still see that over time, the robot's actual position (the Gazebo simulation) drifts from the idealized position in RVIZ. The RVIZ robot finishes the pentagon at almost exactly the same point it started at. This is visualized by the blue dots. In contrast, the Gazebo robot starts at the origin and finishes about a quarter meter from the origin. This is due to the dynamics of the simulation. RVIZ simply displays the odometry data from the robot. These are purely kinematic calculations. The odometry calculations do not model the robot's inertia, the frictional forces in the enviroment, or any other dynamic quality. Maybe a future iteration could incorporate these types of dynamics in order to obtain better state estimations. In contrast, the Gazebo simulation captures these dynamics and results in a realistic simulation of the robot running in real life. The blue ring is the simulated laser data. <br />
!["Gazebo Waypoints"](../../images/gazebo_waypoints.gif)
[![](http://img.youtube.com/vi/eHXuRXVKE6k/0.jpg)](http://www.youtube.com/watch?v=eHXuRXVKE6k "Gazebo and RVIZ Comparison")


# Files in ROS Package
gazebo_waypoints.cpp implements a ROS node to drive the robot to a series of waypoints with a feedback controller.
<br />

turtle_drive_plugin.cpp is the Gazebo plugin. More details and documentation on writing Gazebo plugins can be found [here](http://gazebosim.org/tutorials?tut=plugins_model).
<br />

diff_drive.gazebo.xacro is a xacro file which includes the original xacro file but then adds the extra details needed for a dynamic simulation in Gazebo.
<br />

config/plugin_params.yaml is a file specifying the simulated robot's max velocity and other physical properties.
<br />

config/myWorld.world is a gazebo world
<br />


