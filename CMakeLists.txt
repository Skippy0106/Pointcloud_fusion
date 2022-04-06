cmake_minimum_required(VERSION 3.0.2)
project(ITRI_project)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_ros
  tf2
  tf_conversions
  eigen_conversions
  message_filters
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg
  CATKIN_DEPENDS
  roscpp
  rospy
  pcl_ros
  geometry_msgs
  message_filters
  eigen_conversions
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})
add_executable(pointcloud_merge src/pointcloud_merge.cpp)
target_link_libraries(pointcloud_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES})

