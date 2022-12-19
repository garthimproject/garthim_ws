#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <rl_msgs/DeciderInput.h>
#include <rl_msgs/AgentActionAction.h>
#include <rl_msgs/ChangeReward.h>
/**
 * @brief Node in charge of starting experiments using rosparams.
 * 
 */

namespace rl_starter {
    bool is_gazebo_used_;
    ros::Publisher pub_reset_odom_;
    geometry_msgs::Pose origin;
    bool is_rebootable_, decrement_collision_reward_;
    int decrement_collision_reward_each_exp_;
    int current_experiment_;
    double current_col_rew_;
    std::unique_ptr<actionlib::SimpleActionClient<rl_msgs::AgentActionAction>> ac_;
    void start() {
        ROS_INFO("Wait For Action Server");
        ac_->waitForServer();
        rl_msgs::AgentActionGoal goal;
        goal.action.as_integer.assign(1,-420);
        ac_->sendGoal(goal);
        ROS_INFO("[rl_starter_node] id-experiment %d Sending dummy action %ld.", current_experiment_, goal.action.as_integer[0]);
        current_experiment_++;
    }
    bool rebootExperiment(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        is_rebootable_ = true;
        return true;
    }

    void actualRebootExperiment() {
        if (is_gazebo_used_) {
            gazebo_msgs::SetModelState sms;
            sms.request.model_state.model_name = "turtlebot";
            sms.request.model_state.pose.orientation = origin.orientation;
            sms.request.model_state.pose.position = origin.position;
            sms.request.model_state.twist.linear.x = sms.request.model_state.twist.linear.y 
                = sms.request.model_state.twist.linear.z = 0;
            sms.request.model_state.twist.angular.x =sms.request.model_state.twist.angular.y 
                = sms.request.model_state.twist.angular.z = 0; 
            ROS_INFO("RESET TO %f, %f, %f", origin.position.x,origin.position.y,tf::getYaw(origin.orientation) );
            ros::service::call("/gazebo/set_model_state", sms);
            ros::WallTime t0 = ros::WallTime::now(), t1;
            do {
                pub_reset_odom_.publish(std_msgs::Empty());
                ros::WallRate(20).sleep();
                t1 = ros::WallTime::now();
            } while(ros::ok() && (t1 - t0).toSec() < 2.0);
            ROS_INFO("[rl_starter::rebootExperiment] /gazebo/set_model_state service called and odometry reseted.");
            // Send all nodes to initial state.
            // Restart all file_recorders.
            // decider
            // file_recorder_node
        } else {
            ROS_INFO("[rl_starter::rebootExperiment] Virtual waiting for model.");
            ros::WallDuration(2).sleep();
        }
        std_srvs::Empty msg;
        if(decrement_collision_reward_) {
            ROS_ERROR("Flag to TRUE");
        }
        if (decrement_collision_reward_ && current_experiment_ % decrement_collision_reward_each_exp_ == 0) {
            ROS_ERROR("DECREMENT");
            rl_msgs::ChangeReward r_msg;
            current_col_rew_ += 100;
            r_msg.request.reward = current_col_rew_;
            ros::service::call("change_reward", r_msg);
        } else {ROS_ERROR("NOO DECREMENT");}
        ros::service::call("reset_file_recorder", msg);
        ros::WallDuration(2).sleep();
        ROS_INFO("[rl_starter::rebootExperiment] reset_file_recorder service called");
        ros::service::call("reset_task_interface", msg);
        ros::WallDuration(2).sleep();
        ROS_INFO("[rl_starter::rebootExperiment] reset_task_interface service called");
        ros::service::call("reset_decider", msg);
        ros::WallDuration(2).sleep();
        ROS_INFO("[rl_starter::rebootExperiment] reset_decider service called");
        start();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv,"rl_starter_node");
    ROS_INFO("rl_starter_node STARTED");
    ros::NodeHandle nh, nh_priv("~");
    rl_starter::pub_reset_odom_ = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",10, false);
    ros::ServiceServer service_reboot = nh.advertiseService("reboot_experiment", &rl_starter::rebootExperiment);
    nh_priv.param("robot_pose_x", rl_starter::origin.position.x, 0.0);
    nh_priv.param("robot_pose_y", rl_starter::origin.position.y, -0.352357);
    
    rl_starter::origin.position.z = 0.0;
    double yaw;
    nh_priv.param("robot_pose_yaw", yaw, 0.0);
    rl_starter::origin.orientation = tf::createQuaternionMsgFromYaw(yaw);
    using namespace rl_starter;
    std::vector<int> action_indexes = {0,1,2,3};
    nh.getParam("actions_enabled",action_indexes);
    std_srvs::Empty srv;
    nh.param("decrement_collision_reward", decrement_collision_reward_, false);
    nh.param("decrement_collision_reward_each_step", decrement_collision_reward_each_exp_, 1);
    if (decrement_collision_reward_) {
        nh.param("reward/collision",current_col_rew_, -900.0);
    }
    is_gazebo_used_ = nh.hasParam("gazebo_namespace");
    if (is_gazebo_used_) {
        while(ros::ok() && !ros::service::call("/gazebo/unpause_physics", srv)) {
            ros::Duration(1).sleep();
        }
    }
    rl_starter::current_experiment_ = 0;
    rl_starter::ac_ = std::unique_ptr<actionlib::SimpleActionClient<rl_msgs::AgentActionAction>>(new actionlib::SimpleActionClient<rl_msgs::AgentActionAction>("perform_action", true));
    start();
    is_rebootable_ = false;
    int total_experiments;
    nh.param("total_experiments", total_experiments, 1);
    while (ros::ok() && rl_starter::current_experiment_ < total_experiments) {
        while (ros::ok() && !is_rebootable_) {
            ros::WallDuration(2).sleep();
            ros::spinOnce();
        }
        ros::WallDuration(5).sleep();
        actualRebootExperiment();
        is_rebootable_ = false;
    }
    ROS_INFO("Number of experiments started: %d",  rl_starter::current_experiment_ );
    return 0;
}