#ifndef __DECIDER_EGREEDY_GARTHIM_H__
#define __DECIDER_EGREEDY_GARTHIM_H__
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <decider/decider.h>
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/DeciderInput.h>
#include <decider/loop_detector.h>
namespace decider {
    /**
     * @brief An implementation of a decider that chooses the action based on the maximum value of received value-function.
     * 
     */
    class DeciderMaxPolicy : public Decider {
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
             * @brief Flag indicating if files about loop detection will be written.
             * 
             */
            bool loop_detection_;
            /**
             * @brief Pointer to object for loop detector functionality wrapper.
             * 
             */
            std::unique_ptr<LoopDetector> loop_detector_;
            double thresh_;
        public:
            DeciderMaxPolicy();
            ~DeciderMaxPolicy();
            /**
             * @brief Get the Action index and send it to be performed.
             * 
             * @param d_input It contains value-function for current state. It may also contain current state.
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) override;
        
    };
}
#endif