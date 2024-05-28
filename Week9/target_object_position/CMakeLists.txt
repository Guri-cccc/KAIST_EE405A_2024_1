cmake_minimum_required(VERSION 3.0.2)
project(target_estimation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    cv_bridge
    point_tf
    pcl_ros
    pcl_conversions
    detection_msgs
    point_tf
)
find_package (OpenCV REQUIRED)


catkin_package(
  LIBRARIES opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(target_depth_image_node src/target_depth_image.cpp)
add_dependencies(target_depth_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_depth_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(position_estimation_node src/position_estimation.cpp)
add_dependencies(position_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_estimation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  point_tf_lib
)