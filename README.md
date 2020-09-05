# Navigation Stack 
This is a navigation stack implemented in ROS for a differential drive robot. I am using the Turtlebot3 robot. I am still working on it at the moment! Professor Matt Elwin at Northwestern put together a [course](https://nu-msr.github.io/navigation_site/) for building a navigation stack from scratch. He sketched out what each step should look like and suggested ways of testing. He did not provide any code but did provide a few suggested method signatures. I could not have done it without his guidance, so thanks Matt! 

Below is an image from RVIZ of the diff drive robot navigating to 5 waypoints. In this case, the waypoints form a pentagon <br />
!["Diff Drive Robot Navigating to Waypoints"](images/tbot_pentagon.gif)

# Rigid2D Library
I wrote a C++ library for representing 2D rigid body transformations. It uses screw theory as presented in [Modern Robotics](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) applied to 2D frames.
Describe more of the functions etc   


# Testing
I used the gtest framework for testing my Rigid2D library. This was extremly helpful for debugging and I will definitely use gtest again! 
Describe gtest and the unit tests and where they are etc

# Hardware
I am using the Turtlebot3 Burger. Here is a video of the robot following the same pentagon as in the above gif but in real life!! <br />
!["Turtlebot Pentagon Outside"](images/pentagon.gif)

## TSim Package
Describe

## rigid2d Package
Describe it

## nuturtle_description Package
Describe it  


### Notes to Self 
Remember to put this into the devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.121

