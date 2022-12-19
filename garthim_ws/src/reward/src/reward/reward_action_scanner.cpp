#include <pluginlib/class_list_macros.h>
#include <reward/reward_function.h>
#include <reward/reward_action_scanner.h>

PLUGINLIB_EXPORT_CLASS(reward::RewardActionScannerFunction, reward::RewardFunction)
namespace reward {
    RewardActionScannerFunction::RewardActionScannerFunction() : RewardActionFunction()  {
        ros::NodeHandle nh_priv("~"), nh;
        int scanner_states, spencer_states;
        nh_priv.param("collision", collision_reward_, -100.0);
        nh.param("scanner_states", scanner_states, 1);
        nh.param("spencer_states", spencer_states, 1);
        spatial_states_ /= (scanner_states*spencer_states);
        previous_collision_ = false;
        server_change_ = nh.advertiseService("change_reward", &RewardActionScannerFunction::setCollisionReward, this);
    }
    RewardActionScannerFunction::~RewardActionScannerFunction() {

    }
    double RewardActionScannerFunction::getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) {
        if (next_state.collision) {
            ROS_DEBUG("[ RewardActionScannerFunction::getReward] COLLISION!!!!");
            //return collision_reward_*next_state.collision;
            double col_rew = previous_collision_? 10*collision_reward_: collision_reward_;
            previous_collision_ = true;
            return collision_reward_;
        } else {
            previous_collision_ = false;
        }
        return RewardActionFunction::getReward(state, action, next_state);
    }

    bool RewardActionScannerFunction::setCollisionReward(rl_msgs::ChangeRewardRequest& req, rl_msgs::ChangeRewardResponse& res) {
        collision_reward_ = req.reward;
        return true;
    }
}