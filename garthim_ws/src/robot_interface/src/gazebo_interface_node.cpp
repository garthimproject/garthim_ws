#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_interface/agent_action_server.h>
//#include <robot_interface/experiment_director.h>
int main(int argc, char **argv) {
    ros::init(argc, argv,"gazebo_interface_node");
    ROS_INFO("gazebo_interface_node STARTED");
    robot_interface::AgentSimpleActionServer asas;
    /*ros::MultiThreadedSpinner spinner(3);
    spinner.spin();*/
    ros::spin();
    //robot_interface::SARSA director;
    //director.performExperiment();
    ROS_INFO("gazebo_interface_node ENDING...");
    return 0;
}