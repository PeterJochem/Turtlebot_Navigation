cmake_minimum_required(VERSION 2.8.3)
project(nuturtle_robot)

add_compile_options(-Wall -Wextra -Wno-psabi)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  #nuturtle_robot	
  rigid2d
  nuturtlebot
  tf2_ros
  visualization_msgs
)

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
#  LIBRARIES tsim 
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_srvs tf2_ros visualization_msgs
#  DEPENDS system_lib
)

include_directories(	
       	include
	${catkin_INCLUDE_DIRS}
)

add_executable(turtle_interface src/turtle_interface.cpp)
add_executable(rotation src/rotation.cpp)
add_executable(translate src/translate.cpp)
add_executable(real_waypoint src/real_waypoint.cpp)

set_target_properties(turtle_interface PROPERTIES OUTPUT_NAME turtle_interface PREFIX "")
set_target_properties(rotation PROPERTIES OUTPUT_NAME rotation PREFIX "")
set_target_properties(translate PROPERTIES OUTPUT_NAME translate PREFIX "")
set_target_properties(real_waypoint PROPERTIES OUTPUT_NAME real_waypoint PREFIX "")

add_dependencies(turtle_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rotation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(translate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(real_waypoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(turtle_interface ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(rotation ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(translate ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(real_waypoint ${rigid2d_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
	rotation
	translate
	real_waypoint
	turtle_interface
 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(test_turtle_interface test/interface.test test/test_turtle_interface.cpp)
	target_link_libraries(test_turtle_interface ${rigid2d_LIBRARIES} gtest_main ${catkin_LIBRARIES} ${nuturtlebot_LIBRARIES})
endif()

