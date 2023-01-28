cmake_minimum_required(VERSION 3.0.2)
project(rm_fsm)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    move_base_msgs
    actionlib

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_fsm
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
        include
        state_machine
        lib/smc/lib
        lib/smc/C++
        ${catkin_INCLUDE_DIRS}
)

set(SMC ${PROJECT_SOURCE_DIR}/lib/smc/bin/Smc.jar)
set(SM_CLASS_NAME Patrol)
set(SM_SOURCES ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}.sm)

add_custom_target(${SM_CLASS_NAME}_generate_sm_file
        COMMAND java -jar ${SMC} -c++ ${SM_SOURCES}
        COMMENT "Generating State Machine file")
add_custom_target(${SM_CLASS_NAME}_generate_sm_graph
        COMMAND java -jar ${SMC} -graph -glevel 1 ${SM_SOURCES}
        COMMAND dot -T png ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}_sm.dot -o ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}_sm.png
        COMMENT "Generating State Machine graph")
add_custom_target(${SM_CLASS_NAME}_generate_sm_table
        COMMAND java -jar ${SMC} -table ${SM_SOURCES}
        COMMENT "Generating State Machine graph")

add_dependencies(${SM_CLASS_NAME}_generate_sm_graph ${SM_CLASS_NAME}_generate_sm_file)
add_dependencies(${SM_CLASS_NAME}_generate_sm_table ${SM_CLASS_NAME}_generate_sm_file)

file(GLOB SOURCES
        src/run_smc.cpp
        src/StateMachine.cpp
        src/Patrol.cpp
        state_machine/*.cpp
        )

add_executable(run_smc ${SOURCES})
target_link_libraries(run_smc ${catkin_LIBRARIES})

add_executable(set_point src/set_point.cpp)
target_link_libraries(set_point ${catkin_LIBRARIES})
