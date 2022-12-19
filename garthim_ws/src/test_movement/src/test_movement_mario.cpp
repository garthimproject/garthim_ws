#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
double time_action_ = 1;
double linear, angular;
bool started_ = false;
ros::CallbackQueue queue_;
ros::Subscriber sub;
ros::Publisher pub_cmd_vel_, pubOdom;
nav_msgs::Odometry odometry_twist_;
geometry_msgs::Twist cmd_vel_moving_;
geometry_msgs::Twist cmd_vel_stopping_;
void odomCallback(const  gazebo_msgs::ModelStates::ConstPtr& msg)
{

    started_ = true;
    int crumbIndex = 1;
    for(size_t i =0 ; i< msg->name.size() ;i ++)
    {
        if(msg->name.at(i)=="crumb")
        {
            crumbIndex = i;
        }
    }
    odometry_twist_.pose.pose = msg -> pose.at(crumbIndex);
    odometry_twist_.twist.twist = msg -> twist.at(crumbIndex);
    odometry_twist_.header.frame_id = "odom";
    pubOdom.publish(odometry_twist_);

}
double performAction() {
    ros::Time count = ros::Time::now();
        double start = double(count.toNSec())/1000000000;
        double stop;
        double rate_freq = 100;
        int messages_sent = 0;
        do {
            pub_cmd_vel_.publish(cmd_vel_moving_);
            //ros::Rate(rate_freq).sleep();
            ros::Duration(0.05).sleep();
            count = ros::Time::now();
            stop = double(count.toNSec())/1000000000;
            messages_sent++;
            //ros::Rate(publish_rate_cmd_).sleep();
        } while (ros::ok() && stop - start < time_action_);//(ros::Time::now() - start).toSec() < time_action_);
        ROS_WARN("Messages sent: %d", messages_sent);
        ROS_INFO("[mario::actionComand] loop done. %f s", stop-start);
        pub_cmd_vel_.publish(cmd_vel_stopping_);
        bool eps_x,y,z, wx,wy,eps_z;
        int queue_calls = 0;
        do {
            queue_.callAvailable();
            ros::Duration(0.05).sleep();
            eps_x = abs(odometry_twist_.twist.twist.linear.x) >= 0.001;
            y = abs(odometry_twist_.twist.twist.linear.y) >= 0.001;
            z = abs(odometry_twist_.twist.twist.linear.z) >= 0.001;
            wx = abs(odometry_twist_.twist.twist.angular.x) >= 0.001;
            wy = abs(odometry_twist_.twist.twist.angular.y) >= 0.001;
            eps_z = abs(odometry_twist_.twist.twist.angular.z) >= 0.001;
            queue_calls++;
            //ros::Rate(publish_rate_cmd_).sleep();
        } while (ros::ok() &&(y || z || wx || wy || eps_x || eps_z));// !translator_from_action_.isActionFinished(odometry_twist_));
        //next_state = state_.as_floating[0];
        //steps_++;
        //steps_in_current_episode_++;
        stop = double(count.toNSec())/1000000000;
        ROS_WARN("Queue calls: %d", queue_calls);
        
        return stop - start;//(ros::Time::now() - start).toSec();
}
double actionComand(int action)
{
    //función que comanda un movimiento al CRUMB
    // input: entero con la acción a relaizar
    // sin output

    //determinamos la accion que se nos solicita realizar según el entero "action"
    /*
    remain del significado del valor del entero action:
    default -> stop
    0 -> forward
    1 -> backward
    2 -> turn_left
    3 -> turn_right
    4 -> increase action time
    5 -> decrease action time
    */

    switch(action)
    {
        
        case 0://forward"
        {linear=0.3;angular=0;}
        break;
        case 1://backward"
        {linear=-0.3;angular=0;}
        break;
        case 2://turn_left"
        {linear=0;angular=0.9;}
        break;
        case 3://turn_right
        {linear=0;angular=-0.9;}
        break;
        default:
        {linear=0;angular=0;}
    }
    return 0;
}
int main(int argc, char **argv) {
    ros::init(argc, argv,"test_movement_mario");
    ROS_INFO("test_movement_mario STARTED");
    ros::NodeHandle nh;
    nh.setCallbackQueue(&queue_);
    srand(0);
    sub = nh.subscribe("/gazebo/model_states",1, &odomCallback);
    pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100, false);
    pubOdom = nh.advertise<nav_msgs::Odometry>("mario",100, false);
    cmd_vel_moving_.linear.x =cmd_vel_moving_.linear.y =cmd_vel_moving_.linear.z = 0;
    cmd_vel_moving_.angular.x = cmd_vel_moving_.angular.y = cmd_vel_moving_.angular.z = 0;
    cmd_vel_stopping_.linear.x = cmd_vel_stopping_.linear.y = cmd_vel_stopping_.linear.z = 0;
    cmd_vel_stopping_.angular.x = cmd_vel_stopping_.angular.y = cmd_vel_stopping_.angular.z = 0;
    while (ros::ok()) {
        if (!started_) {
            ros::WallDuration(1).sleep();
            queue_.callAvailable();
            continue;
        }
        int num = rand()%4;
        ROS_INFO("Random %d", num);
        actionComand(num);
        cmd_vel_moving_.linear.x = linear;
        cmd_vel_moving_.angular.z = angular;
        performAction();
    }
}