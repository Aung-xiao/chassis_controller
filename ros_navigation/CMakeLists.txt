cmake_minimum_required(VERSION 3.0.2)
project(ros_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)
find_package(PCL REQUIRED) 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_navigation
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_odom src/basic_odom.cpp)
target_link_libraries(basic_odom ${catkin_LIBRARIES})

add_executable(pointcloud2_filter src/pointcloud2_filter.cpp)
target_link_libraries(pointcloud2_filter
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
