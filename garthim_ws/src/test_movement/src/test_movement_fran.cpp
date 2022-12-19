#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/callback_queue.h>
ros::Publisher pubOdom;
ros::Subscriber sub;
nav_msgs::Odometry _odom;
ros::CallbackQueue queue_;
double linear, angular;
bool started_ = false;
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
}
void odomCallback(const  gazebo_msgs::ModelStates::ConstPtr& msg)
{
    started_ = true;
    int crumbIndex = 1;
    for(size_t i =0 ; i< msg->name.size() ;i ++)
    {
        if(msg->name.at(i)=="turtlebot3_burger")
        {
            crumbIndex = i;
        }
    }
    _odom.pose.pose = msg -> pose.at(crumbIndex);
    _odom.twist.twist = msg -> twist.at(crumbIndex);
    _odom.header.frame_id = "odom";
    pubOdom.publish(_odom);

}
int main(int argc, char **argv) {
    ros::init(argc, argv,"test_movement_fran");
    srand(0);
    ROS_INFO("gazebo_interface_node STARTED");
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("");

    //nh.setCallbackQueue(&queue_);
    sub = nh.subscribe("/gazebo/model_states",1, &odomCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100, false);
    pubOdom = nh.advertise<nav_msgs::Odometry>("francisco_spin_once",100, false);
    geometry_msgs::Twist sentMessage;
    sentMessage.angular.x = sentMessage.angular.y = sentMessage.angular.z = 0;
    sentMessage.linear.x = sentMessage.linear.y = sentMessage.linear.z = 0;
    double _actionTime = 1.0;
    while (ros::ok()) {
        if (!started_) {
            ros::WallDuration(1).sleep();
            //queue_.callAvailable();//ros::spinOnce();
            ros::spinOnce();
            continue;
        }
        int num = rand()%4;
        ROS_INFO("Random %d", num);
        actionComand(num);
    sentMessage.linear.x=linear;
    
    sentMessage.angular.z=angular;

        ros::Time count = ros::Time::now();
        double start = double(count.toNSec())/1000000000;
        double stop = start;
        double duration = 0.05;
        double rate_freq = 100;
        ROS_INFO("[PhysicalHandle::actionComand] Beginning loop. %f s", _actionTime);
        
        int messages_sent = 0;
    if (ros::ok()) {
            do {
                //ROS_INFO("[PhysicalHandle::actionComand] time:%d, vx=%f, vz=%f", _actionTime, linear, angular);
                pub.publish(sentMessage);
                ros::Duration(duration).sleep();
                //ros::Rate(rate_freq).sleep();
                count = ros::Time::now();
                stop = double(count.toNSec())/1000000000;
                messages_sent++;
                //ROS_INFO("[PhysicalHandle::actionComand] sleep done");
            } while ((stop-start)< _actionTime && ros::ok());
        }
        ROS_WARN("Messages sent: %d", messages_sent);
        ROS_INFO("[PhysicalHandle::actionComand] loop done. %f s", stop-start);
        sentMessage.linear.x=0;
        sentMessage.angular.z=0;
        pub.publish(sentMessage);
        bool eps_x,y,z, wx,wy,eps_z;
        int queue_calls = 0;
        
        //Wait for message.
        if (ros::ok()) {
            do {
                ros::spinOnce();
                //queue_.callAvailable();
                eps_x = abs(_odom.twist.twist.linear.x) >= 0.001;
                y = abs(_odom.twist.twist.linear.y) >= 0.001;
                z = abs(_odom.twist.twist.linear.z) >= 0.001;
                wx = abs(_odom.twist.twist.angular.x) >= 0.001;
                wy = abs(_odom.twist.twist.angular.y) >= 0.001;
                eps_z = abs(_odom.twist.twist.angular.z) >= 0.001;
                ros::Duration(duration).sleep();
                queue_calls++;
            } while(ros::ok() && (y || z || wx || wy || eps_x || eps_z));
        }
        stop = double(count.toNSec())/1000000000;
        ROS_WARN("Queue calls: %d", queue_calls);
        
        double neededTime =stop-start;
    }
    ROS_INFO("gazebo_interface_node ENDING...");
    return 0;
}