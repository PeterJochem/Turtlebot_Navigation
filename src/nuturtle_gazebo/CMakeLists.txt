cmake_minimum_required(VERSION 2.8.3)
project(nuturtle_gazebo)

add_compile_options(-Wall -Wextra -Wno-psabi)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nuturtlebot
  gazebo_ros
  rigid2d
  tf2_ros
  visualization_msgs 
)

find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   start.srv
   start_waypoints.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nuturtle_gazebo 
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_srvs tf2_ros visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_waypoints src/gazebo_waypoints.cpp)
add_executable(gazebo_motion_planning src/gazebo_motion_planning.cpp)

set_target_properties(gazebo_waypoints PROPERTIES OUTPUT_NAME gazebo_waypoints PREFIX "")
set_target_properties(gazebo_motion_planning PROPERTIES OUTPUT_NAME gazebo_motion_planning PREFIX "")

add_dependencies(gazebo_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_motion_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_waypoints ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(gazebo_motion_planning ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(turtle_drive_plugin SHARED src/turtle_drive_plugin.cpp)
target_link_libraries(turtle_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_include_directories(turtle_drive_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
add_dependencies(turtle_drive_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
