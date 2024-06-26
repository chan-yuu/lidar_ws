cmake_minimum_required(VERSION 2.8.3)
project(lidarObstacleDetect)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  perception_msgs

)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES lidar_detect
  CATKIN_DEPENDS roscpp rospy std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  perception_msgs
 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#link_directories(
 # lib
 # ${catkin_LIB_DIRS}
#)

# 添加子目录并构建
add_subdirectory(lib/pre_process/roi_clip)
add_subdirectory(lib/pre_process/voxel_grid_filter)
add_subdirectory(lib/cluster/euclideanCluster)
add_subdirectory(lib/bounding_box)
add_subdirectory(lib/ground_detector/patchwork)
add_subdirectory(lib/visualization)

add_executable(${PROJECT_NAME}_node src/lidar_detection_track.cpp src/lidar_detection_track_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  roi_clip
  voxel_grid_filter
  euclideanCluster
  bounding_box
  patchwork
  visualization

 )


