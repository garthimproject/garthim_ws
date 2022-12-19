#ifndef __REWARD_GARTHIM_REWARD_ACTION_FUNCTION_H__
#define __REWARD_GARTHIM_REWARD_ACTION_FUNCTION_H__
#include <ros/ros.h>
#include <reward/reward_function.h>
namespace reward {
    class RewardActionFunction : public RewardFunction {
        private:
            std::vector<double> limits_;
            std::vector<std::vector<double>> rewards_per_action_;
        protected:
            uint64_t spatial_states_;
            int time_action_indexes_;
        public:
            RewardActionFunction();
            ~RewardActionFunction();
            double getReward(rl_msgs::State state,
                rl_msgs::RLVariable action, rl_msgs::State next_state) override;
            void readRewardMatrixFromParam(const std::vector<double>& rewards,const int num_limits, const int num_actions);
            void getRewardsPerActionVector(std::vector<std::vector<double>>& rewards_per_action);
    };
}
#endif