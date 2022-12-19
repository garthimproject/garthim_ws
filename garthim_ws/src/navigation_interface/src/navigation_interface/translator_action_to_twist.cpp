#include <navigation_interface/translator_action_to_twist.h>

namespace navigation_interface {
    TranslatorActionToTwist::TranslatorActionToTwist() {
        ros::NodeHandle nh("~"), nh_global;
        double time_action;
        action_indexes_ = {0,1,2,3};
        nh_global.getParam("actions_enabled",action_indexes_);
        this->total_actions_ = action_indexes_.size();
        nh.param("linear_velocity_module",this->linear_velocity_module_,1.0);
        nh.param("angular_velocity_module",this->angular_velocity_module_,1.0);
        action_durations_ = {1.0};
        nh.getParam("times_for_action",action_durations_);
        id_current_action_duration_ = action_durations_.size() / 2;
        is_discrete_ = nh.hasParam("discrete");
        // SEND TIME ACTION TODO
    }
    TranslatorActionToTwist::~TranslatorActionToTwist() {}
    bool TranslatorActionToTwist::translateDiscrete(const rl_msgs::RLVariable& input,
             geometry_msgs::Twist& translation) {
        translation.linear.x = translation.linear.y = translation.linear.z = 0;
        translation.angular.x = translation.angular.y = translation.angular.z = 0;
        int action =input.as_integer[0] < 0?-1: action_indexes_[input.as_integer[0]];
        switch (action) {
            case -1:
                translation.linear.x = -this->linear_velocity_module_;
                ROS_ERROR("[TranslatorActionToTwist::translateDiscrete] Recovery behaviour selected.");
                break;
            case 0:
                translation.linear.x = this->linear_velocity_module_;
                break;
            case 1:
                translation.linear.x = -this->linear_velocity_module_;
                break;
            case 2:
                // anti-clockwise turn
                translation.angular.z = this->angular_velocity_module_;
                break;
            case 3:
                // clock-wise turn
                translation.angular.z = -this->angular_velocity_module_;
                break;
            case 4:
                id_current_action_duration_ = 
                    (id_current_action_duration_ + 1) % action_durations_.size();
                break;
            case 5:
                id_current_action_duration_ -= 1;
                id_current_action_duration_ = id_current_action_duration_ < 0?action_durations_.size()-1:id_current_action_duration_;
                break;
            default:
                ROS_FATAL("[TranslatorActionToTwist::translateDiscrete] Only %d operations supported.",
                   this->total_actions_ );
        }
        return action < 4;
    }
    bool TranslatorActionToTwist::translateContinuous(const rl_msgs::RLVariable& input,
             geometry_msgs::Twist& translation) {
        ROS_FATAL("[TranslatorActionToTwist::translateContinuous] Not Implemented");
        return false;
    }
}