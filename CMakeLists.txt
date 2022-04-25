cmake_minimum_required(VERSION 3.0.2)
project(itri_project)


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
  pcl_conversions
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

add_executable(L515_merge src/L515_merge.cpp)
target_link_libraries(L515_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
