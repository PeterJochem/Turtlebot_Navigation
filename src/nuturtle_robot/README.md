# Description 
This package implements ROS nodes to make the robot navigate in the real world. 

# Results
Here is a video of the robot navigating to a series of waypoints in the real world. The waypoints form a pentagon in this case. <br /> 
!["Turtlebot Pentagon Outside"](../..images/pentagon.gif)

# How to Run my Code
FIX ME - running code on real robot versus running it in RVIZ/simulation
```roslaunch nuturtle_robot follow_waypoints.launch robot:=1``` to make the robot navigate to waypoins in the real world.
```roslaunch nuturtle_robot follow_waypoints.launch robot:=1``` to make the robot navigate to waypoins in RVIZ.
```roslaunch nuturtle_robot test_movement.launch``` to make the robot do 20 rotations in the real world.


# Files in ROS Package
turtle_interface.cpp is a ROS node that serves as a low layer link to OpenCR and the actual hardware. It listens for twists for the robot to perform and converts this to the corresponding wheel commands (ie motor commands). It also takes sensor_data and repackages it as easier to use forms. For example, it takes the encoder ticks values and converts them to the angles of the wheels.
<br />

real_waypoint.cpp is a ROS node which makes the robot navigate to a pentagon of waypoints in the real world.
<br />

rotation.cpp is a ROS node which has the robot rotate 20 times in the real world. Each rotation is followed by a brief pause.  
<br />

translate.cpp is a ROS node which has the robot do a few short translations followed by a brief pause.
<br />


