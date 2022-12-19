cmake_minimum_required(VERSION 2.8.3)
project(navigation_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rl_msgs
  robot_interface
  random_numbers
  roscpp
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  kobuki_msgs
  spencer_tracking_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   rl_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation_interface
  CATKIN_DEPENDS pluginlib rl_msgs robot_interface roscpp std_msgs tf tf2_geometry_msgs random_numbers
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(target_generator
  src/${PROJECT_NAME}/target_generator.cpp
)
add_dependencies(target_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(target_generator
    ${catkin_LIBRARIES}
  )
add_library(target_generator_fixed
  src/${PROJECT_NAME}/target_generator_fixed.cpp
)
add_dependencies(target_generator_fixed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_generator_fixed
    target_generator
    ${catkin_LIBRARIES}
)
add_library(target_generator_inside_region
  src/${PROJECT_NAME}/target_generator_inside_region.cpp
)
add_dependencies(target_generator_inside_region ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_generator_inside_region
    target_generator
    ${catkin_LIBRARIES}
)

add_library(target_generator_inside_region_fixed
  src/${PROJECT_NAME}/target_generator_inside_region_fixed.cpp
)
add_dependencies(target_generator_inside_region ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_generator_inside_region_fixed
    target_generator
    ${catkin_LIBRARIES}
)

add_library(target_generator_scaffolder
  src/${PROJECT_NAME}/target_generator_scaffolder.cpp
)
add_dependencies(target_generator_scaffolder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_generator_scaffolder
    target_generator_inside_region
    ${catkin_LIBRARIES}
)

add_library(target_generator_simple_obstacle
  src/${PROJECT_NAME}/target_generator_simple_obstacle.cpp
)
add_dependencies(target_generator_simple_obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_generator_simple_obstacle
    target_generator
    ${catkin_LIBRARIES}
)

add_library(translator_to_state
  src/${PROJECT_NAME}/translator_to_state.cpp
)
add_dependencies(translator_to_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(translator_to_state
    target_generator_scaffolder
    ${catkin_LIBRARIES}
  )
add_library(translator_action_to_twist
  src/${PROJECT_NAME}/translator_action_to_twist.cpp
)
add_dependencies(translator_action_to_twist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(translator_action_to_twist
    ${catkin_LIBRARIES}
  )


## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/navigation_interface.cpp
 )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
    translator_action_to_twist
    ${catkin_LIBRARIES}
    translator_to_state
  )
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(scanner_node_test src/scanner_node_test.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(scanner_node_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(scanner_node_test
   translator_to_state
   ${catkin_LIBRARIES}
 )

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

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_getter_gazebo_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)


#  add_rostest_gtest(navigation_interface_test
#  launch/navigation_interface_test.test
#  test/gtest_main.cpp
#  test/navigation_interface/navigation_interface_test.cpp
#  )
#  if(TARGET navigation_interface_test)
#  find_package(rostest)
#  target_link_libraries(navigation_interface_test  ${catkin_LIBRARIES} navigation_interface)
#  endif()


  add_rostest_gtest(translator_scan_to_state_test
  launch/translator_scan_to_state_test.test
  test/gtest_main.cpp
  test/navigation_interface/translator_scan_to_state_test.cpp
  )
  if(TARGET translator_scan_to_state_test)
  find_package(rostest)
  target_link_libraries(translator_scan_to_state_test  ${catkin_LIBRARIES} translator_to_state)
  endif()
endif()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
