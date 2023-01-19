cmake_minimum_required(VERSION 3.0.2)
project(l3_footstep_planning_msgs)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++17, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  tf
  actionlib_msgs
  actionlib
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  vigir_generic_params
  l3_msgs
  l3_libs
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES} message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EditStep.msg
  ErrorStatus.msg
  FeetPoseRequest.msg
  PatternGeneratorParameters.msg
  PatternParameters.msg
  PlanningFeedback.msg
  PlanningStatistics.msg
  StepPlanFeedback.msg
  StepPlanRequest.msg
  StepPlan.msg
  UpdateMode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EditStepService.srv
  GenerateFeetPoseService.srv
  GetStepPlanService.srv
  PatternGeneratorParametersService.srv
  SetStepPlanService.srv
  StepPlanRequestService.srv
  StitchStepPlanService.srv
  TransformFeetPosesService.srv
  TransformFootPoseService.srv
  TransformStepPlanService.srv
  UpdateFeetService.srv
  UpdateFootService.srv
  UpdateStepPlanService.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  EditStep.action
  ExecuteStepPlan.action
  GenerateFeetPose.action
  GetStepPlan.action
  SetStepPlan.action
  StepPlanRequest.action
  StitchStepPlan.action
  UpdateFeet.action
  UpdateFoot.action
  UpdateStepPlan.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  l3_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES} message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/footstep_planning_msgs.h
)

set(SOURCES
  src/footstep_planning_msgs.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
# add_executable(l3_footstep_planning_msgs_node src/l3_footstep_planning_msgs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.msg"
)

install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.srv"
)

install(DIRECTORY action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.action"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_l3_footstep_planner_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
