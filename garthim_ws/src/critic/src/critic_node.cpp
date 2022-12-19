#include <ros/ros.h>
#include <critic/critic.h>
#include <pluginlib/class_loader.h>
#include <memory>
int main(int argc, char **argv) {
    ros::init(argc, argv,"critic_node");
    ROS_INFO("critic node STARTED");

    ros::NodeHandle nh("~");
    pluginlib::ClassLoader<critic::Critic> c_loader_("critic", "critic::Critic");
    boost::shared_ptr<critic::Critic> critic_;
    std::string critic_instance_name =  "not implemented";
    critic_instance_name = nh.param("critic_implementation", critic_instance_name);
    try {
        critic_ = c_loader_.createInstance(critic_instance_name);
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the \"%s\" critic plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
             critic_instance_name.c_str(), ex.what());
            exit(1);
    }

    ros::spin();
    ROS_INFO("critic node ENDING...");
    return 0;
}