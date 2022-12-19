#include <pluginlib/class_list_macros.h>
#include <reward/reward_function.h>
#include <reward/reward_next_state_function.h>

PLUGINLIB_EXPORT_CLASS(reward::RewardNextStateFunction, reward::RewardFunction)
namespace reward {
    RewardNextStateFunction::RewardNextStateFunction() {
        ros::NodeHandle nh_priv("~"), nh;
        int total_states, time_action_indexes, scanner_states;
        limits_ = {0, 8};
        rewards_ = {100,0};
        nh_priv.getParam("bounds", limits_);
        nh_priv.getParam("rewards", rewards_);
        ROS_INFO("%f %f", limits_[0], limits_[1]);
        ROS_INFO("%f %f", rewards_[0], rewards_[1]);
        nh.param("total_states", total_states,64);
        nh.param("time_actions_states", time_action_indexes, 1);
        spatial_states_ = (uint64_t) (total_states/time_action_indexes);
        ROS_WARN("[RewardNextStateFunction::RewardNextStateFunction] spatial_states_: %lu", spatial_states_);
    }
    RewardNextStateFunction::~RewardNextStateFunction() {

    }
    double RewardNextStateFunction::getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) {

        const double state_var = next_state.content.as_floating.size()>0? next_state.content.as_floating[0]
            : next_state.content.as_integer[0]%spatial_states_;
        int id = std::lower_bound(limits_.begin(), limits_.end(), state_var)
            - limits_.begin();
        if (id >= rewards_.size() || id > 0 && state_var < limits_[id]) {
            id = id -1;
        }
        return id == 0 && state_var < limits_[id] ? 0.0 : this->rewards_[id];
    }
}