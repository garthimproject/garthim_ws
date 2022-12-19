#ifndef __DECIDER_LOOP_EVASION_GARTHIM_H__
#define __DECIDER_LOOP_EVASION_GARTHIM_H__
#include <decider/decider_boltzmann_correction.h>
#include <decider/loop_detector.h>
#include <file_recorder/file_recorder.h>
#include <unordered_set>
namespace decider {
    /**
     * @brief A sub-class of DeciderBoltzmannCorrection that implements an ad-hoc way of detecting state loops.
     * ([1] - Angel Martínez-Tenor, Juan Antonio Fernández-Madrigal, Ana Cruz-Martín and Javier González-Jiménez, 
     * Towards a common implementation of reinforcement learning for multiple robotics tasks. February 2017.
     * DOI.  10.1016/j.eswa.2017.11.011)
     * BY CONVENTION, FIRST CALL IS NOT PART OF THE LEARNING PROCESS SO IT MAY BE IGNORED FOR LOOP DETECTION.
     */
    class DeciderLoopEvasion : public DeciderBoltzmannCorrection {
        private:
            /**
             * @brief Pointer to loop detector wrapper.
             * 
             */
            std::unique_ptr<LoopDetector> loop_detector_;
        public:
            DeciderLoopEvasion();
            ~DeciderLoopEvasion();
            /**
             * @brief Get the Action index and send it to be performed.
             * 
             * @param d_input It contains value-function for current state. It may also contain current state.
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) override;

    };
}
#endif