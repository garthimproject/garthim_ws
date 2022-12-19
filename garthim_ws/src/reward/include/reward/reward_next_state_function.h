#ifndef __REWARD_GARTHIM_REWARD_NEXT_STATE_FUNCTION_H__
#define __REWARD_GARTHIM_REWARD_NEXT_STATE_FUNCTION_H__
#include <ros/ros.h>
#include <reward/reward_function.h>
namespace reward {
    /**
     * @brief A class for a reward function implementation that only looks at the next_state of a transition.
     * 
     */
    class RewardNextStateFunction : public RewardFunction {
        private:
            /**
             * @brief A vector containing the states that have a change in reward in this reward function.
             * 
             */
            std::vector<double> limits_;
            /**
             * @brief A vector containing the reward values associated to the limits.
             * 
             */
            std::vector<double> rewards_;
        protected:
            /**
             * @brief Total states without considerating obstacles.
             * 
             */
            uint64_t spatial_states_;
        public:
            RewardNextStateFunction();
            ~RewardNextStateFunction();
            /**
             * @brief Obtain the reward finding next_state in vector limits_ and checking the value of the interval in rewards_.
             * 
             * @param state Current state.
             * @param action Current action.
             * @param next_state State obtained after performing current action from current state.
             * @return double Reward value linked to next_state.
             */
            double getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) override;
        
    };
}
#endif