#include <model_interface/model_interface.h>
int main(int argc, char **argv) {
    ros::init(argc, argv,"model_interface_node");
    ROS_INFO("model_interface node STARTED");
    model_interface::ModelInterface mo;
    ros::spin();
    return 0;
}