sudo rm -r arm_build/ arm_devel/ arm_install/
sudo rm -r build devel
./arm catkin_arm install
rsync -av --delete arm_install/ student@turtlebot1:/home/student/install
catkin_make
