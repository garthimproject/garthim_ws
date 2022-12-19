#ifndef __REWARD_GARTHIM_REWARD_FUNCTION_H__
#define __REWARD_GARTHIM_REWARD_FUNCTION_H__
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/State.h>
namespace reward {
    /**
     * @brief Abstract class (interface) for a reward function.
     * 
     */
    class RewardFunction {
        public:
            virtual ~RewardFunction() {}
            /**
             * @brief Reward obtained from input.
             * 
             * @param state Current state of the transition.
             * @param action Action choosen from current state.
             * @param next_state State obtained after performing action.
             * @return double reward obtained from input.
             */
            virtual double getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) = 0;
        protected:
            RewardFunction() {}
    };
}
#endif