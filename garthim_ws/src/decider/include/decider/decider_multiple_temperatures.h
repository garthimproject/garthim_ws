#ifndef __DECIDER_MULTIPLE_TEMPERATURES_GARTHIM_H__
#define __DECIDER_MULTIPLE_TEMPERATURES_GARTHIM_H__
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <decider/decider_boltzmann_correction.h>
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/GetRandom.h>
#include <numeric>
namespace decider { 
    /**
     * @brief A sub-class of DeciderBoltzmannCorrection that keeps a different temperature value for each state.
     * 
     */
    class DeciderMultipleTemperatures : public DeciderBoltzmannCorrection {
        private:
            /**
             * @brief A vector containing each temperature value for its state.
             * 
             */
            std::vector<double> temperatures_;
        public:
            DeciderMultipleTemperatures();
            ~DeciderMultipleTemperatures();
            /**
             * @brief Get the Action index and send it to be performed.
             * 
             * @param d_input It contains value-function for current state. It may also contain current state.
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) override;
    };
}
#endif