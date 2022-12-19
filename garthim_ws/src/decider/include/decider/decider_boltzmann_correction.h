#ifndef __DECIDER_BOLTZMANN_CORRECTION_GARTHIM_H__
#define __DECIDER_BOLTZMANN_CORRECTION_GARTHIM_H__
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <decider/decider.h>
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/GetRandom.h>
#include <numeric>
namespace decider {
    /**
     * @brief An implementation of a Decider using a softmax strategy for choosing the action to be performed.
     * This class assumes that in each iteration the value-function for current state is received.
     * With these values, the exponential distribution is obtained first correcting each value substracting the maximum 
     * value in value-function and second is divided by a temperature value, which is lowering itself each iteration by 
     * lambda unities.
     * 
     */
    class DeciderBoltzmannCorrection : public Decider {
        protected:
            /**
             * @brief Current decrement of temperature_.
             * 
             */
            double lambda_;
            /**
             * @brief This value is used to regulate the significance of each value of a value-function for
             * a state when computing its exponential distribution.
             * 
             */
            double temperature_;
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
            random_numbers::RandomNumberGenerator ran_gen_;
        public:
            DeciderBoltzmannCorrection();
            ~DeciderBoltzmannCorrection();
            /**
             * @brief Get the Action index and send it to be performed. It delegates in getOldAction.
             * 
             * @param d_input It contains value-function for current state. It may also contain current state.
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) override;
            /**
             * @brief Get the Action index and send it to be performed. This method is called inside getAction.
             * 
             * @param sv_function value-function for current state.
             * @param temp Current temperature value.
             */
            void getOldAction(const std::vector<double> &sv_function, const double temp);
            /**
             * @brief This methods obtains a random number from a node which provides a centralized ros architecture rng.
             * 
             * @return int random
             */
            static int getRandom();
    };
}
#endif