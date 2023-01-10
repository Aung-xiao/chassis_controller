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
  state_machine
  lib/smc/lib
  ${catkin_INCLUDE_DIRS}
)

set(SMC ${PROJECT_SOURCE_DIR}/lib/smc/bin/Smc.jar)
set(SM_CLASS_NAME EasyStateMachine_fsm)
set(SM_SOURCES ${PROJECT_SOURCE_DIR}/state_machine/simple_state.sm)

add_custom_target(EasyStateMachine_generate_sm_file
        COMMAND java -jar ${SMC} -c++ ${SM_SOURCES}
        COMMENT "Generating State Machine file")


add_library(chassis_controller
        src/chassis_controller.cpp
        )

target_link_libraries(chassis_controller
        ${catkin_LIBRARIES}
        )

add_library(steering_chassis_controller
        src/steering_chassis_controller.cpp
        )
target_link_libraries(steering_chassis_controller
        ${catkin_LIBRARIES}
        )

file(GLOB SOURCES
        src/run_smc.cpp
        src/EasyStateMachine.cpp
        state_machine/*.cpp
        )

add_executable(run_smc ${SOURCES})
target_link_libraries(run_smc ${catkin_LIBRARIES})


