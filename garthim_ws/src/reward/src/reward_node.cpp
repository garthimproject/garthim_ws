#include <ros/ros.h>
#include <reward/reward.h>
#include <memory>
int main(int argc, char **argv) {
    ros::init(argc, argv,"reward_node");
    ROS_INFO("reward node STARTED");
    reward::RewardGenerator gen;
    ros::spin();
    ROS_INFO("reward node ENDING...");
    return 0;
}