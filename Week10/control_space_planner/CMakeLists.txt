cmake_minimum_required(VERSION 2.8.3)
project(control_space_planner)

# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES a_star_library
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  tf
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include
)

add_executable(control_space_planner_node src/control_space_planner_node.cpp)
add_dependencies(control_space_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_space_planner_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )