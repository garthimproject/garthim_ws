#ifndef __LOOP_DETECTOR_GARTHIM_H__
#define __LOOP_DETECTOR_GARTHIM_H__
#include <file_recorder/file_recorder.h>
#include <rl_msgs/DeciderInput.h>
#include <unordered_set>
namespace decider {
    class LoopDetector {
        private:
            /**
             * @brief (Debugging purposes) This is the filename used to store the queue states relevant when detecting loops.
             * 
             */
            std::string filename_queue_states_;
            /**
             * @brief(Debugging purposes) This is the filename used to store the value-function received in each step.
             * 
             */
            std::string filename_q_s_;
            /**
             * @brief (Debugging purposes) This is the filename used to store the relevant rewards received in each step.
             * 
             */
            std::string filename_reward_states_;
            /**
             * @brief (Debugging purposes) This is the filename used to store the current steps.
             * 
             */
            std::string filename_steps_;
            /**
             * @brief List containing all states relevant to detect a loop.
             * 
             */
            std::list<int64_t> queue_states_;
            /**
             * @brief A set containing all unique states relevant to detect a loop.
             * 
             */
            std::unordered_set<int64_t> set_states_;
            /**
             * @brief A list containing all rewards for each state in queue_states_
             * 
             */
            std::list<double> queue_rewards_;
            /**
             * @brief A flag that indicates whether this is the first call of the decider. 
             * 
             */
            bool is_first_call_;
            /**
             * @brief A flag indicating whether first reward in a queue is yet to be obtained. 
             * 
             */
            bool is_first_reward_;
            /**
             * @brief A flag indicating if temperature decreases during learning or stays constant.
             * 
             */
            bool is_decreasing_;
            /**
             * @brief Average of queue_rewards_.
             * 
             */
            double reward_avg_;
            /**
             * @brief Initial value of temperature that may or may not decrease depending on is_decreasing_ flag.
             * 
             */
            double initial_temperature_;
            /**
             * @brief Current temperature value taking into account increments in Loop Detection.
             * 
             */
            double current_temperature_;
            /**
             * @brief Factor of decrement of temperature.
             * 
             */
            double lambda_;
            /**
             * @brief Number of values inside queue_states_ to initiate loop evasion.
             * 
             */
            size_t min_size_;
            /**
             * @brief Number of values that at maximum may be inside loop evasion. When a new value is received, first value is dequeued.
             * 
             */
            size_t max_size_;
            /**
             * @brief Number of loop decisions made since start.
             * 
             */
            uint64_t steps_;
            /**
             * @brief A wrapper for storing debugging info.
             * 
             */
            file_recorder::FileRecorder file_recorder_;
        public:
            LoopDetector(double temperature, double lambda);
            LoopDetector();
            ~LoopDetector();
            double detectLoop(const rl_msgs::DeciderInput &d_input);

    };

}
#endif