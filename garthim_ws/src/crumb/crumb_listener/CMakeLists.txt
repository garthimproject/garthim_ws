cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(crumb_listener)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  arbotix_python
  roscpp
  rospy
  std_msgs
  kobuki_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crumb_home
#  CATKIN_DEPENDS arbotix_python roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(  ${catkin_INCLUDE_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


add_executable(crumb_listener_gazebo src/crumb_listener_gazebo.cpp)
target_link_libraries(crumb_listener_gazebo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})
target_link_libraries(crumb_listener_gazebo   ${catkin_LIBRARIES} )





