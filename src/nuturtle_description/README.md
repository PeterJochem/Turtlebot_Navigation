# Description
This ROS package describes a differential drive robot. There are xacro files as well as a launchfile to visualize the robot's geometry in RVIZ.

## How to View Robot
Run ```roslaunch nuturtle_description view_diff_drive.launch``` with (optional) arguments use_jsp_gui - this runs the joint state publisher with its GUI and lets the user manipulate the joint angles of the robot

### Files in Package
config/diff_params.yaml - description of the robot's dimensions <br />

diff_drive.rviz - RVIZ file for robot visualization <br />

launch/view_diff_drive.launch - launch file for viewing the robot <br />

diff_drive.urdf.xacro - xacro file for creating urdf of the differential drive robot 

