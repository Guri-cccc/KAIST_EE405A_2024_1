cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_projection)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
    cv_bridge
)
find_package (OpenCV REQUIRED)

# catkin_package(
#   LIBRARIES
# )

catkin_package(
  LIBRARIES opencv
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(pointcloud_projection_node src/pointcloud_projection.cpp)
add_dependencies(pointcloud_projection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_projection_node
  ${catkin_LIBRARIES}
)