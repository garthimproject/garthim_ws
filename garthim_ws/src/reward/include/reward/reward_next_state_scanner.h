#ifndef __REWARD_GARTHIM_REWARD_NEXT_STATE_SCANNER_FUNCTION_H__
#define __REWARD_GARTHIM_REWARD_NEXT_STATE_SCANNER_FUNCTION_H__
#include <ros/ros.h>
#include <reward/reward_next_state_function.h>
#include <rl_msgs/ChangeReward.h>
namespace reward {
    /**
     * @brief A sub-class of RewardNextStateFunction that accounts for collisions.
     * 
     */
    class RewardNextStateScannerFunction : public RewardNextStateFunction {
        private:
            /**
             * @brief Reward given to collisions.
             * 
             */
            double collision_reward_;
            ros::ServiceServer server_change_;
        public:
            RewardNextStateScannerFunction();
            ~RewardNextStateScannerFunction();
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