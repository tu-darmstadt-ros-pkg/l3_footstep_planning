cmake_minimum_required(VERSION 3.0.2)
project(l3_footstep_planning_vis_tools)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++17, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  visualization_msgs
  robot_state_publisher
  vigir_pluginlib
  l3_msgs
  l3_libs
  l3_vis
  l3_plugins
  l3_footstep_planning_msgs
  l3_footstep_planning_libs
  l3_footstep_planning_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

# Compiler definitions and options
add_l3_compile_definitions_and_options()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/base/planning_markers_vis_plugin.h
  include/${PROJECT_NAME}/base/planning_vis_plugin.h
  include/${PROJECT_NAME}/std/ground_contact_estimation_vis.h
  include/${PROJECT_NAME}/std/inverse_kinematics_vis.h
  include/${PROJECT_NAME}/std/planning_feedback_vis.h
  include/${PROJECT_NAME}/std/step_plan_path_vis.h
  include/${PROJECT_NAME}/std/step_plan_request_vis.h
  include/${PROJECT_NAME}/std/step_plan_vis.h
  include/${PROJECT_NAME}/std/upper_body_vis.h
  include/${PROJECT_NAME}/footstep_planning_vis_node.h
  include/${PROJECT_NAME}/visualization.h
)

set(SOURCES
  src/base/planning_markers_vis_plugin.cpp
  src/base/planning_vis_plugin.cpp
  src/std/ground_contact_estimation_vis.cpp
  src/std/inverse_kinematics_vis.cpp
  src/std/planning_feedback_vis.cpp
  src/std/step_plan_path_vis.cpp
  src/std/step_plan_request_vis.cpp
  src/std/step_plan_vis.cpp
  src/std/upper_body_vis.cpp
  src/visualization.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(footstep_planning_vis_node src/footstep_planning_vis_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(footstep_planning_vis_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} footstep_planning_vis_node
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
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

# Mark plugin description file for installation
install(FILES
  plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
