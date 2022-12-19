#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <nav_msgs/Odometry.h>
#include <rl_msgs/GetRandom.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher pub_;
ros::Subscriber sub_odom_, sub_target_;
std::string robot_model_name_, frame_id_, child_frame_id_, service_goal_gazebo_name_;
nav_msgs::Odometry msg_;
inline void doOdomTask(const gazebo_msgs::GetModelStateResponse& response) {
    //auto pose = response.pose;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
        
    transformStamped.header.stamp = response.header.stamp;
    transformStamped.header.frame_id = frame_id_;
    transformStamped.child_frame_id = child_frame_id_;
    transformStamped.transform.translation.x = response.pose.position.x;
    transformStamped.transform.translation.y = response.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = response.pose.orientation.x;
    transformStamped.transform.rotation.y = response.pose.orientation.y;
    transformStamped.transform.rotation.z = response.pose.orientation.z;
    transformStamped.transform.rotation.w = response.pose.orientation.w;

    br.sendTransform(transformStamped);
    msg_.child_frame_id = child_frame_id_;
    msg_.header.frame_id = frame_id_;
    msg_.header.stamp = transformStamped.header.stamp;
    msg_.pose.pose = response.pose;
    msg_.pose.covariance.fill(0);
    msg_.twist.twist = response.twist;
    msg_.twist.covariance.fill(0);
    pub_.publish(msg_);
}
void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for(int i = msg->name.size() - 1; i>=0;i--) {
        if(msg->name.at(i)==robot_model_name_){
            auto pose = msg->pose[i];
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = frame_id_;
            transformStamped.child_frame_id = child_frame_id_;
            transformStamped.transform.translation.x = pose.position.x;
            transformStamped.transform.translation.y = pose.position.y;
            transformStamped.transform.translation.z = 0.0;
            transformStamped.transform.rotation.x = pose.orientation.x;
            transformStamped.transform.rotation.y = pose.orientation.y;
            transformStamped.transform.rotation.z = pose.orientation.z;
            transformStamped.transform.rotation.w = pose.orientation.w;

            br.sendTransform(transformStamped);
            msg_.child_frame_id = child_frame_id_;
            msg_.header.frame_id = frame_id_;
            msg_.header.stamp = transformStamped.header.stamp;
            msg_.pose.pose = msg->pose[i];
            msg_.pose.covariance.fill(0);
            msg_.twist.twist = msg->twist[i];
            msg_.twist.covariance.fill(0);
            pub_.publish(msg_);
            break;
        }
    }
}

void sendTargetToGazebo(const geometry_msgs::PoseStamped& target) {
    gazebo_msgs::SetModelState model_pose;

        model_pose.request.model_state.model_name="TargetMark5";
        model_pose.request.model_state.pose.position.x = target.pose.position.x;
        model_pose.request.model_state.pose.position.y = target.pose.position.y;
        model_pose.request.model_state.pose.position.z = 0;
        model_pose.request.model_state.pose.orientation.w = 1;
        model_pose.request.model_state.pose.orientation.x = 0;
        model_pose.request.model_state.pose.orientation.y = 0;
        model_pose.request.model_state.pose.orientation.z = 0;
        model_pose.request.model_state.twist.linear.x = 0.0;
        model_pose.request.model_state.twist.linear.y = 0.0;
        model_pose.request.model_state.twist.linear.z = 0.0;
        model_pose.request.model_state.twist.angular.x = 0.0;
        model_pose.request.model_state.twist.angular.y = 0.0;
        model_pose.request.model_state.twist.angular.z = 0.0;
        model_pose.request.model_state.reference_frame = "world";
        int count = 0;
        while (count < 3 && !ros::service::call(service_goal_gazebo_name_.c_str(),model_pose)) {
            ROS_WARN("[rl_gazebo_node::sendTargetToGazebo] TARGET SERVICE FAIL, RETRYING...");
            ros::Duration(1).sleep();
            count++;
        }
}

bool getRandom(rl_msgs::GetRandom::Request& req, rl_msgs::GetRandom::Response& res) {
    res.random_number = rand();
    return true;
}

int main(int argc, char **argv) {
    srand(time(0));
    ros::init(argc, argv,"rl_gazebo_node");
    ROS_INFO("rl_gazebo_node STARTED");
    ros::NodeHandle nh, nh_priv("~");
    frame_id_="odom";
    child_frame_id_="base_footprint";
    std::string namespace_gazebo = nh.param("gazebo_namespace",std::string("gazebo"));
    std::string odom_gazebo_topic = "/"+namespace_gazebo+"/model_states";
    service_goal_gazebo_name_ = "/"+namespace_gazebo+"/set_model_state";
    robot_model_name_ = nh.param("robot_model_name", std::string("crumb"));
    sub_target_ = nh.subscribe("target", 1,&sendTargetToGazebo);
    //gazebo_goal_sender_ = nh.serviceClient<gazebo_msgs::SetModelState>(service_goal_gazebo_name.c_str());
    
    ros::ServiceServer server_ = nh.advertiseService("get_random", &getRandom);
    
    bool publish_odom;
    nh_priv.param("publish_odom", publish_odom, false);
    if (publish_odom) {
        bool service;
        pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100000, false);
        nh_priv.param("call_service",service, false);
        if (service) {
            gazebo_msgs::GetModelStateRequest req;
            gazebo_msgs::GetModelStateResponse res;
            ros::ServiceClient sc = nh.serviceClient<gazebo_msgs::GetModelState>("/"+namespace_gazebo+"/get_model_state",true);
            req.model_name = robot_model_name_;
            while(ros::ok()) {
                if (sc.isValid()) {
                    if (sc.call(req, res)) {
                        doOdomTask(res);
                    }
                } else {
                    sc.shutdown();
                    sc = nh.serviceClient<gazebo_msgs::GetModelState>("/"+namespace_gazebo+"/get_model_state",true);
                }
                ros::spinOnce();
                ros::Rate(1000).sleep();
            }
        } else {
            sub_odom_ = nh.subscribe(odom_gazebo_topic.c_str(), 100000,&receiveGazeboModelStates);
        }
    } 
    ros::spin();

    ROS_INFO("rl_gazebo_node ENDING...");
    return 0;
}
