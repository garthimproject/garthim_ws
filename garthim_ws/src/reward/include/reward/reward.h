#ifndef __REINFORCEMENT_LEARNING_GARTHIM_REWARD_H__
#define __REINFORCEMENT_LEARNING_GARTHIM_REWARD_H__
#include <ros/ros.h>
#include <memory>
#include <rl_msgs/GetReward.h>
#include <reward/reward_function.h>
#include <pluginlib/class_loader.h>
namespace reward {
    /**
     * @brief A Service server provider wrapping the usage of a reward function.
     * 
     */
    class RewardGenerator {
        private:
            /**
             * @brief A server instance to be advertise through ros network.
             * 
             */
            ros::ServiceServer server_;
            /**
             * @brief A reward function class loader. It must be alive during execution or else 
             * function may cause the node to crash.
             * 
             */
            pluginlib::ClassLoader<RewardFunction> rf_loader_;
            /**
             * @brief A pointer to reward function interface.
             * 
             */
            boost::shared_ptr<RewardFunction> function_;
        public:
            RewardGenerator();
            ~RewardGenerator();
            /**
             * @brief Service callback, from a request composed of a state, an action and a next_state it returns the
             * reward associated to them.
             * 
             * @param req Tuple (s,a,S) (state, action, next_state)
             * @param res (out-parameter) reward value
             * @return true if service call has ended correctly.
             * @return false otherwise.
             */
            virtual bool getReward(rl_msgs::GetRewardRequest& req, 
            rl_msgs::GetRewardResponse& res);
    };
}

#endif