#ifndef __REWARD_GARTHIM_REWARD_ACTION_SCANNER_FUNCTION_H__
#define __REWARD_GARTHIM_REWARD_ACTION_SCANNER_FUNCTION_H__
#include <ros/ros.h>
#include <reward/reward_action_function.h>
#include <rl_msgs/ChangeReward.h>
namespace reward {
    /**
     * @brief A sub-class of RewardNextStateFunction that accounts for collisions.
     * 
     */
    class RewardActionScannerFunction : public RewardActionFunction {
        private:
            /**
             * @brief Reward given to collisions.
             * 
             */
            double collision_reward_;
            bool previous_collision_;
            ros::ServiceServer server_change_;
        public:
            RewardActionScannerFunction();
            ~RewardActionScannerFunction();
            /**
             * @brief Checks if next_state has a collision and then calls super-method. 
             * (Super-method) Obtain the reward finding next_state in vector limits_ and 
             * checking the value of the interval in rewards_.
             * 
             * @param state Current state.
             * @param action Current action.
             * @param next_state State obtained after performing current action from current state.
             * @return double Reward value linked to next_state.
             */
            double getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) override;

            bool setCollisionReward(rl_msgs::ChangeRewardRequest& req, rl_msgs::ChangeRewardResponse& res);
        
    };
}
#endif