#ifndef __CRITIC_GARTHIM_H__
#define __CRITIC_GARTHIM_H__
#include <rl_msgs/TransitionResult.h>
namespace critic {
    /**
     * @brief Generic class (Interface) for Critic entity, which is tasked to create the AI (value function) for RL algorithm.
     * To work properly in a ROS architecture, the implemented class has to subscribe to a topic using updateValueFunction()
     */
    class Critic {
        public:
            virtual ~Critic() {}
            /**
             * @brief Generic method to update a value function. The implementation of the value function is done under subclass.
             * By convention, when updateValueFunction no longer updates the function, learning is considered to have finished.
             * @param tr <const rl_msgs::TransitionResult&> The current Transition message sent.
             */
            virtual void updateValueFunction(const rl_msgs::TransitionResult& tr) = 0;
            /**
             * @brief Generic method to load a previously learnt value function. This can be done either to renew a learning process 
             * under different conditions or to just use a constant value function without modifying it.
             */
            virtual void loadValueFunction() = 0;
        protected:
            Critic() {}
    };
}
#endif