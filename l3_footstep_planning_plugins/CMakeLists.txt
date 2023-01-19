cmake_minimum_required(VERSION 3.0.2)
project(l3_footstep_planning_plugins)

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
  std_msgs
  nav_msgs
  cv_bridge
  pcl_conversions
  vigir_generic_params
  vigir_pluginlib
  l3_plugins
  l3_footstep_planning_msgs
  l3_footstep_planning_libs
  l3_terrain_model
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/aggregator/extended_plugin_aggregator.h
  include/${PROJECT_NAME}/aggregator/heuristic.h
  include/${PROJECT_NAME}/aggregator/hfs_heuristic.h
  include/${PROJECT_NAME}/aggregator/post_processor.h
  include/${PROJECT_NAME}/aggregator/reachability.h
  include/${PROJECT_NAME}/aggregator/state_generator.h
  include/${PROJECT_NAME}/aggregator/step_cost_estimator.h
  include/${PROJECT_NAME}/aggregator/use_mask_generator.h
  include/${PROJECT_NAME}/aggregator/world_model.h
  include/${PROJECT_NAME}/base/collision_check_grid_map_plugin.h
  include/${PROJECT_NAME}/base/footstep_planning_plugin.h
  include/${PROJECT_NAME}/base/heuristic_plugin.h
  include/${PROJECT_NAME}/base/hfs_heuristic_plugin.h
  include/${PROJECT_NAME}/base/state_generator_plugin.h
  include/${PROJECT_NAME}/base/post_process_plugin.h
  include/${PROJECT_NAME}/base/reachability_plugin.h
  include/${PROJECT_NAME}/base/step_cost_estimator_plugin.h
  include/${PROJECT_NAME}/base/terrain_model_plugin.h
  include/${PROJECT_NAME}/base/use_mask_generator_plugin.h
  include/${PROJECT_NAME}/base/world_model_plugin.h
  include/${PROJECT_NAME}/std/heuristic/dynamics_heuristic.h
  include/${PROJECT_NAME}/std/heuristic/heading_heuristic.h
  include/${PROJECT_NAME}/std/heuristic/euclidean_heuristic.h
  include/${PROJECT_NAME}/std/heuristic/occupancy_grid_map_heuristic.h
  include/${PROJECT_NAME}/std/heuristic/step_cost_heuristic.h
  include/${PROJECT_NAME}/std/heuristic/travel_time_heuristic.h
  include/${PROJECT_NAME}/std/post_processor/ground_contact_post_process.h
  include/${PROJECT_NAME}/std/post_processor/simple_floating_base_post_process.h
  include/${PROJECT_NAME}/std/post_processor/step_dynamics_post_process.h
  include/${PROJECT_NAME}/std/robot_model/state_3d_reachability.h
  include/${PROJECT_NAME}/std/robot_model/ik_reachability.h
  include/${PROJECT_NAME}/std/robot_model/dynamics_reachability.h
  include/${PROJECT_NAME}/std/robot_model/polygonal_reachability.h
  include/${PROJECT_NAME}/std/robot_model/support_polygon_check.h
  include/${PROJECT_NAME}/std/state_generator/direct_goal_state_generator.h
  include/${PROJECT_NAME}/std/state_generator/expand_state_job.h
  include/${PROJECT_NAME}/std/state_generator/ik_polygonal_state_generator.h
  include/${PROJECT_NAME}/std/state_generator/lattice_goal_state_generator.h
  include/${PROJECT_NAME}/std/state_generator/lattice_state_generator.h
  include/${PROJECT_NAME}/std/state_generator/polygonal_state_generator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/boundary_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/const_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/diagonal_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/euclidean_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/ground_contact_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/travel_time_step_cost_estimator.h
  include/${PROJECT_NAME}/std/use_mask_generator/occ_grid_use_mask_generator.h
  include/${PROJECT_NAME}/std/step_range_polygon.h
  include/${PROJECT_NAME}/std/terrain_model/basic_terrain_model.h
  include/${PROJECT_NAME}/std/world_model/foot_grid_map_model.h
  include/${PROJECT_NAME}/std/world_model/grid_map_2d.h
  include/${PROJECT_NAME}/std/world_model/grid_map_model.h
  include/${PROJECT_NAME}/std/world_model/upper_body_grid_map_model.h
)

set(SOURCES
  src/aggregator/heuristic.cpp
  src/aggregator/hfs_heuristic.cpp
  src/aggregator/post_processor.cpp
  src/aggregator/reachability.cpp
  src/aggregator/state_generator.cpp
  src/aggregator/step_cost_estimator.cpp
  src/aggregator/use_mask_generator.cpp
  src/aggregator/world_model.cpp
  src/base/collision_check_grid_map_plugin.cpp
  src/base/footstep_planning_plugin.cpp
  src/base/heuristic_plugin.cpp
  src/base/hfs_heuristic_plugin.cpp
  src/base/post_process_plugin.cpp
  src/base/reachability_plugin.cpp
  src/base/state_generator_plugin.cpp
  src/base/step_cost_estimator_plugin.cpp
  src/base/terrain_model_plugin.cpp
  src/base/use_mask_generator_plugin.cpp
  src/base/world_model_plugin.cpp
  src/std/heuristic/dynamics_heuristic.cpp
  src/std/heuristic/heading_heuristic.cpp
  src/std/heuristic/euclidean_heuristic.cpp
  src/std/heuristic/occupancy_grid_map_heuristic.cpp
  src/std/heuristic/step_cost_heuristic.cpp
  src/std/heuristic/travel_time_heuristic.cpp
  src/std/post_processor/ground_contact_post_process.cpp
  src/std/post_processor/simple_floating_base_post_process.cpp
  src/std/post_processor/step_dynamics_post_process.cpp
  src/std/robot_model/state_3d_reachability.cpp
  src/std/robot_model/ik_reachability.cpp
  src/std/robot_model/dynamics_reachability.cpp
  src/std/robot_model/polygonal_reachability.cpp
  src/std/robot_model/support_polygon_check.cpp
  src/std/state_generator/direct_goal_state_generator.cpp
  src/std/state_generator/expand_state_job.cpp
  src/std/state_generator/ik_polygonal_state_generator.cpp
  src/std/state_generator/lattice_goal_state_generator.cpp
  src/std/state_generator/lattice_state_generator.cpp
  src/std/state_generator/polygonal_state_generator.cpp
  src/std/step_cost_estimator/boundary_step_cost_estimator.cpp
  src/std/step_cost_estimator/const_step_cost_estimator.cpp
  src/std/step_cost_estimator/diagonal_step_cost_estimator.cpp
  src/std/step_cost_estimator/euclidean_step_cost_estimator.cpp
  src/std/step_cost_estimator/ground_contact_step_cost_estimator.cpp
  src/std/step_cost_estimator/travel_time_step_cost_estimator.cpp
  src/std/use_mask_generator/occ_grid_use_mask_generator.cpp
  src/std/step_range_polygon.cpp
  src/std/terrain_model/basic_terrain_model.cpp
  src/std/world_model/foot_grid_map_model.cpp
  src/std/world_model/grid_map_2d.cpp
  src/std/world_model/grid_map_model.cpp
  src/std/world_model/upper_body_grid_map_model.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

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
