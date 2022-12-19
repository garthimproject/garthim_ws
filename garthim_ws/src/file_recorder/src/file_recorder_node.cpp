#include <file_recorder/file_recorder.h>

#include <ros/ros.h>
#include <memory>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
namespace file_recorder_node {
    std::shared_ptr<file_recorder::FileRecorder> f_pointer_;
    ros::Subscriber sub, sub_scaff;
    inline void init() {
        ros::NodeHandle nh, nh_priv("~");
        f_pointer_ = 
        std::shared_ptr<file_recorder::FileRecorder>(new file_recorder::FileRecorder(false));
        sub = nh.subscribe("transition_result",100,&file_recorder::FileRecorder::record, file_recorder_node::f_pointer_.get());
        if (nh_priv.hasParam("filename_scaffolding")) {
            ROS_WARN("[file_recorder_node] Subscribing to scaffolding topic in order to record.");
            sub_scaff = nh.subscribe("scaffolding", 100, &file_recorder::FileRecorder::recordScaffold, 
            file_recorder_node::f_pointer_.get());
        }
    }
    bool reset(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        //init();
        ros::shutdown();
        return true;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv,"file_recorder_node");
    ROS_INFO("file_recorder node STARTED");
    file_recorder_node::init();
    ros::NodeHandle nh;
    ros::ServiceServer reset = nh.advertiseService("reset_file_recorder", &file_recorder_node::reset);
    //ros::ServiceServer getWriters = nh.advertiseService("get_writable_file_recorder", &file_recorder_node::getWritableFileRecorder);
    //f = std::move(file_recorder::FileRecorder(false));
    ros::spin();
    ROS_INFO("file_recorder node ENDING...");
    return 0;
}