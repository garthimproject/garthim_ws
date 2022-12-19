#ifndef __DECIDER_EGREEDY_GARTHIM_H__
#define __DECIDER_EGREEDY_GARTHIM_H__
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <decider/decider.h>
#include <rl_msgs/RLVariable.h>
namespace decider {
    /**
     * @brief A class that implements an epsilon-greedy strategy to choose an action based on value-function. 
     * 
     */
    class DeciderEGreedy : public Decider {
        private:
            /**
             * @brief A variable to store current action to be sent to be performed. Stored for non-serialized connections.
             * 
             */
            rl_msgs::RLVariable action_;
            /**
             * @brief Subscriber to Critic topic.
             * 
             */
            ros::Subscriber sub_sv_function_;
            /**
             * @brief Epsilon parameter loaded and used to decide action.
             * 
             */
            double epsilon_;
            /**
             * @brief RNG wrapper inside a ros package. A new distinct seed is assigned each time an object 
             * is created without its initialization params.
             * 
             */
            random_numbers::RandomNumberGenerator ran_gen_;
        public:
            DeciderEGreedy();
            ~DeciderEGreedy();
            /**
             * @brief Get the Action index and send it to be performed.
             * 
             * @param d_input It contains value-function for current state. It also contains current state.
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) override;
        
    };
}
#endif