cmake_minimum_required(VERSION 3.0.2)
project(l3_footstep_planning_libs)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++17, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf l3_footstep_planning_msgs l3_libs l3_math l3_plugins)

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
  LIBRARIES l3_footstep_planning_libs
  CATKIN_DEPENDS roscpp tf l3_footstep_planning_msgs l3_libs l3_math l3_plugins
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/modeling/discrete_action.h
  include/${PROJECT_NAME}/modeling/floating_base_database.h
  include/${PROJECT_NAME}/modeling/floating_base_id.h
  include/${PROJECT_NAME}/modeling/floating_base_step_action.h
  include/${PROJECT_NAME}/modeling/foothold_database.h
  include/${PROJECT_NAME}/modeling/foothold_id.h
  include/${PROJECT_NAME}/modeling/foot_step_action.h
  include/${PROJECT_NAME}/modeling/planning_state.h
  include/${PROJECT_NAME}/modeling/state_space_manager.h
  include/${PROJECT_NAME}/modeling/state.h
  include/${PROJECT_NAME}/modeling/state_database.h
  include/${PROJECT_NAME}/modeling/state_id.h
  include/${PROJECT_NAME}/modeling/step_plan.h
  include/${PROJECT_NAME}/modeling/transition.h
  include/${PROJECT_NAME}/modeling/transition_database.h
  include/${PROJECT_NAME}/modeling/transition_id.h
  include/${PROJECT_NAME}/threading/threading_manager.h
  include/${PROJECT_NAME}/threading/queue.h
  include/${PROJECT_NAME}/threading/worker.h
  include/${PROJECT_NAME}/hash.h
  include/${PROJECT_NAME}/helper.h
  include/${PROJECT_NAME}/math.h
  include/${PROJECT_NAME}/typedefs.h
)

set(SOURCES
  src/modeling/floating_base_database.cpp
  src/modeling/floating_base_id.cpp
  src/modeling/floating_base_step_action.cpp
  src/modeling/foot_step_action.cpp
  src/modeling/foothold_database.cpp
  src/modeling/foothold_id.cpp
  src/modeling/planning_state.cpp
  src/modeling/state_space_manager.cpp
  src/modeling/state.cpp
  src/modeling/state_database.cpp
  src/modeling/state_id.cpp
  src/modeling/step_plan.cpp
  src/modeling/transition.cpp
  src/modeling/transition_database.cpp
  src/modeling/transition_id.cpp
  src/hash.cpp
  src/helper.cpp
  src/math.cpp
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

## Specify additional locations of test files
set(TEST_HEADERS
  test/include/${PROJECT_NAME}/macros.h
  test/include/${PROJECT_NAME}/msgs_test_helper.h
)

set(TEST_SOURCES
  test/src/floating_base_hashing_utest.cpp
  test/src/msgs_test_helper.cpp
  test/src/foothold_database_utest.cpp
  test/src/foothold_hashing_utest.cpp
  test/src/math_utest.cpp
  test/src/state_database_utest.cpp
  test/src/state_hashing_utest.cpp
  test/src/step_plan_utest.cpp
  test/src/transition_database_utest.cpp
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  add_library(${PROJECT_NAME}-test ${TEST_SOURCES} ${TEST_HEADERS})
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

  catkin_add_gtest(utest test/src/utest.cpp ${TEST_SOURCES})

  if(TARGET ${PROJECT_NAME}-test)
    target_include_directories(${PROJECT_NAME}-test PUBLIC test/include ${GMOCK_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(utest ${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
