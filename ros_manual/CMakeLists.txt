cmake_minimum_required(VERSION 3.0.2)
project(ros_manual)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rm_common
  rm_msgs
  roscpp
  roslint
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_manual
#  CATKIN_DEPENDS nav_msgs rm_common rm_msgs roscpp roslint serial std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
FILE(GLOB ALL_SOURCES "src/*.cpp" )
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )