
#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
  std::cout << "Running main() from gtest_main.cc\n";
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv,"robot_interface_tests");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}