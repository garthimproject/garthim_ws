#include <reward/reward.h>

namespace reward {
    RewardGenerator::RewardGenerator() : rf_loader_("reward", "reward::RewardFunction") {
        ros::NodeHandle nh;
        std::string reward_function_instance_name =  "reward::RewardNextStateFunction";
        reward_function_instance_name = nh.param("reward_function", reward_function_instance_name);
        try {
            function_ = rf_loader_.createInstance(reward_function_instance_name);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s function, are you sure it is properly registered and that the containing library is built? Exception: %s",
             reward_function_instance_name.c_str(), ex.what());
            exit(1);
        }
        server_ = nh.advertiseService("get_reward", &RewardGenerator::getReward, this);
    }

    RewardGenerator::~RewardGenerator() {

    }

    bool RewardGenerator::getReward(rl_msgs::GetRewardRequest& req, 
        rl_msgs::GetRewardResponse& res) {
        res.reward = function_->getReward(req.state,req.action,req.next_state);
        return true;
    }
}