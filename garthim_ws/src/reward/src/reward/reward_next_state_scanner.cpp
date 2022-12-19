#include <pluginlib/class_list_macros.h>
#include <reward/reward_function.h>
#include <reward/reward_next_state_scanner.h>

PLUGINLIB_EXPORT_CLASS(reward::RewardNextStateScannerFunction, reward::RewardFunction)
namespace reward {
    RewardNextStateScannerFunction::RewardNextStateScannerFunction() : RewardNextStateFunction()  {
        ros::NodeHandle nh_priv("~"), nh;
        int scanner_states, spencer_states;
        nh_priv.param("collision", collision_reward_, -100.0);
        nh.param("scanner_states", scanner_states, 1);
        nh.param("spencer_states", spencer_states, 1);
        spatial_states_ /= (scanner_states*spencer_states);
        server_change_ = nh.advertiseService("change_reward", &RewardNextStateScannerFunction::setCollisionReward, this);
    }
    RewardNextStateScannerFunction::~RewardNextStateScannerFunction() {

    }
    double RewardNextStateScannerFunction::getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) {
        if (next_state.collision) {
            ROS_DEBUG("[ RewardNextStateScannerFunction::getReward] COLLISION!!!!");
            //return collision_reward_*next_state.collision;
            return collision_reward_;
        }
        return RewardNextStateFunction::getReward(state, action, next_state);
    }
    bool RewardNextStateScannerFunction::setCollisionReward(rl_msgs::ChangeRewardRequest& req, rl_msgs::ChangeRewardResponse& res) {
        collision_reward_ = req.reward;
        return true;
    }
}