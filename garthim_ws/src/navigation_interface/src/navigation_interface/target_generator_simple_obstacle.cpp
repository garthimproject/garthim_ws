#include <navigation_interface/target_generator_simple_obstacle.h>
#include <gazebo_msgs/SetModelState.h>
namespace navigation_interface {
    TargetGeneratorSimpleObstacle::TargetGeneratorSimpleObstacle() {
        origin_.position.x =origin_.position.y =origin_.position.z = 0;
        origin_.orientation.x = origin_.orientation.y = origin_.orientation.z = 0;
        origin_.orientation.w = 1;

        /*destiny_.position.x = 1.15;
        destiny_.position.y = -1;
        destiny_.position.z = 0;
        destiny_.orientation.x = destiny_.orientation.y = destiny_.orientation.z = 0;
        destiny_.orientation.w = 1;*/
        current_id_path_ = 0;
        path_.push_back(geometry_msgs::Pose());
        path_.back().position.x = 0;
        path_.back().position.y = -0.60;
        path_.back().position.z = 0;
        path_.back().orientation.x = path_.back().orientation.y = path_.back().orientation.z = 0;
        path_.back().orientation.w = 1;

       /* path_.push_back(geometry_msgs::Pose());
        path_.back().position.x = 0;
        path_.back().position.y = -1;
        path_.back().position.z = 0;
        path_.back().orientation.x = path_.back().orientation.y = path_.back().orientation.z = 0;
        path_.back().orientation.w = 1;
*/
        path_.push_back(geometry_msgs::Pose());
        path_.back().position.x = 0.5;
        path_.back().position.y = -0.60;
        path_.back().position.z = 0;
        path_.back().orientation.x = path_.back().orientation.y = path_.back().orientation.z = 0;
        path_.back().orientation.w = 1;

        path_.push_back(geometry_msgs::Pose());
        path_.back().position.x = 1;
        path_.back().position.y = -0.60;
        path_.back().position.z = 0;
        path_.back().orientation.x = path_.back().orientation.y = path_.back().orientation.z = 0;
        path_.back().orientation.w = 1;
    }
    TargetGeneratorSimpleObstacle::~TargetGeneratorSimpleObstacle(){}

    bool TargetGeneratorSimpleObstacle::getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status/* = 0*/) {
            if(status==1 || current_id_path_ + 1 == path_.size()) {
                /*std_srvs::Empty resetWorldSrv;
                ros::service::call("/gazebo/reset_world", resetWorldSrv);
                ROS_INFO("[TargetGeneratorSimpleObstacle::getNewTarget] RESTARTING SIMULATION");*/
                gazebo_msgs::SetModelState sms;
                sms.request.model_state.model_name = "turtlebot";
                sms.request.model_state.pose.orientation = origin_.orientation;
                sms.request.model_state.pose.position = origin_.position;
                sms.request.model_state.twist.linear.x = sms.request.model_state.twist.linear.y 
                    = sms.request.model_state.twist.linear.z = 0;
                sms.request.model_state.twist.angular.x =sms.request.model_state.twist.angular.y =
                sms.request.model_state.twist.angular.z = 0; 
                ros::service::call("/gazebo/set_model_state", sms);
                for (auto &p:path_) {
                    p.position.y = -p.position.y;
                }
                current_id_path_ = 0;
                //destiny_.position.y = -destiny_.position.y;
            } else if (status==2) {
                current_id_path_++;
            }
            target.position.x = path_[current_id_path_].position.x;
            target.position.y = path_[current_id_path_].position.y;
            target.position.z = 0;
        return false;
    }
}