cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  hardware_interface
  pluginlib
  roscpp
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chassis_controller
#  CATKIN_DEPENDS control_toolbox controller_interface hardware_interface pluginlib roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(chassis_controller
        include/chassis_controller/chassis_controller.h
        src/chassis_controller.cpp
                )
target_link_libraries(chassis_controller
        ${catkin_LIBRARIES}
        )

add_library(steering_chassis_controller
        include/chassis_controller/steering_chassis_controller.h
        src/steering_chassis_controller.cpp
        )
target_link_libraries(steering_chassis_controller
        ${catkin_LIBRARIES}
        )