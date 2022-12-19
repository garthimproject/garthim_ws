#include <pluginlib/class_list_macros.h>
#include <reward/reward_function.h>
#include <reward/reward_action_function.h>

PLUGINLIB_EXPORT_CLASS(reward::RewardActionFunction, reward::RewardFunction)
namespace reward {
    RewardActionFunction::RewardActionFunction() {
        ros::NodeHandle nh_priv("~"), nh;
        std::vector<double> rewards;
        int total_states, scanner_states;
        limits_ = {0, 8};
        rewards = {100,0};
        nh_priv.getParam("bounds", limits_);
        bool ok = nh_priv.getParam("rewards_as_vector", rewards);
        if (!ok) {
            ROS_ERROR("Default params used in reward values");
        } else {
            std::stringstream ss;
            for (const auto& r : rewards) {
                ss << r<<" ";
            }
            ROS_INFO("REWARDS: %s", ss.str().c_str());
        }
        ROS_INFO("%f %f", limits_[0], limits_[1]);
        nh.param("total_states", total_states,64);
        nh.param("time_actions_states", time_action_indexes_, 1);
        spatial_states_ = (uint64_t) (total_states/time_action_indexes_);
        ROS_WARN("[RewardActionFunction::RewardActionFunction] spatial_states_: %lu", spatial_states_);
        ////
        std::vector<int> action = {0,2,3};
        nh.getParam("actions_enabled",action);
        readRewardMatrixFromParam(rewards, limits_.size(), action.size());
    }
    RewardActionFunction::~RewardActionFunction() {

    }
    double RewardActionFunction::getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) {
        const uint64_t spatial_time_states = spatial_states_ * time_action_indexes_;
        const int id_time = (state.content.as_integer[0]%spatial_time_states)/spatial_states_;
        const double state_var = next_state.content.as_floating.size()>0? next_state.content.as_floating[0]
            : next_state.content.as_integer[0]%spatial_states_; //TODO CHANGE
        const int action_var = action.as_floating.empty()? action.as_integer[0] : action.as_floating[0]; // TODO CONTINUOUS
        int id = std::lower_bound(limits_.begin(), limits_.end(), state_var)
            - limits_.begin();
        
        if (id >= rewards_per_action_[0].size() || id > 0 && state_var < limits_[id]) {
            id = id -1;
        }
        // assert(action_var < rewards_per_action_.size());
        const float r_fac = 1;//action_var > 2 || id == 0? 1: 2.0/(id_time+1);
        return id == 0 && state_var < limits_[id] ? 0.0 : this->rewards_per_action_[action_var][id]/r_fac;
    }
    void RewardActionFunction::readRewardMatrixFromParam(const std::vector<double>& rewards, const int num_limits, const int num_actions) {
        if (rewards.size()%num_limits!=0  || rewards.size()%num_actions!=0) {
            ROS_ERROR("[RewardActionFunction::readRewardMatrixFromParam] param rewards_as_vector not properly set. \nsize: %lu, limits: %d, num_actions: %d", 
                rewards.size(), num_limits,num_actions);
            throw std::runtime_error("[RewardActionFunction::readRewardMatrixFromParam] param rewards_as_vector not properly set.");
        }
        const auto it_begin = rewards.begin();
        for (auto it = rewards.begin(); it != rewards.end(); it++) {
            if (std::distance(it_begin, it) % num_limits == 0) {
                this->rewards_per_action_.emplace_back();
            }
            rewards_per_action_.back().push_back(*it);
        }
        if (rewards_per_action_.size()!=num_actions) {
            throw std::runtime_error("[RewardActionFunction::readRewardMatrixFromParam] param rewards_as_vector not properly set.");
        }
    }
    
    void RewardActionFunction::getRewardsPerActionVector(std::vector<std::vector<double>>& rewards_per_action) {
        rewards_per_action = rewards_per_action_;
    }
}