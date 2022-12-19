#include <ros/ros.h>
#include <rl_metrics/rl_metrics.h>
#include <std_srvs/Empty.h>
int main(int argc, char **argv) {
    ros::init(argc, argv,"rl_metrics_node");
    ROS_INFO("rl_metrics_node STARTED");
    rl_metrics::MeasureQ mq;
    mq.runExperiments();

    // reiniciamos el mundo gazebo para poder realizar una nueva simulacion
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
    //ros::NodeHandle().setParam("rl_metrics_start", false);

    ROS_INFO("rl_metrics_node ENDING...");
    ros::WallDuration(20).sleep();
    return 0;
}