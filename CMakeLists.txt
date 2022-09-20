cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_fusion)


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
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
  # If yaml-cpp not found in the system, try finding it as a user CMake-generated project
  FIND_PACKAGE(yaml-cpp REQUIRED)
  INCLUDE_DIRECTORIES(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(pointcloud_merge_new src/pointcloud_merge_new.cpp)
target_link_libraries(pointcloud_merge_new ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(extrinsic_yaml src/extrinsic_yaml.cpp)
target_link_libraries(extrinsic_yaml ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

