#ifndef __TASK_INTERFACE_ROBOT_INTERFACE_ROBOT_INTERFACE_H__
#define __TASK_INTERFACE_ROBOT_INTERFACE_ROBOT_INTERFACE_H__
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/State.h>
namespace robot_interface {
    /**
     * @brief A Generic Class (Interface) to abstract the particularities of a Task and the physical agent that performs it
     * from the rest of the nodes of the stack.
     * 
     */
    class TaskInterface {
        public:
            virtual ~TaskInterface() {}
            /**
             * @brief Obtain the current state of the agent.
             * 
             * @param state out-parameter containing a representation of the physical state of the agent.
             */
            virtual void getState(rl_msgs::State& state) = 0;
            /**
             * @brief A method for asking the physical agent to perform actions in the real world.
             * 
             * @param action An abstraction of the action to be performed.
             * @return double The performance time of the action.
             */
            virtual double performAction(const rl_msgs::RLVariable& action) = 0;
            /**
             * @brief A method to reset the state after the execution of an episode.
             * 
             * @param status Feedback about the status of the robot after the episode. Ex: not-finished, aborted, success.
             * @return true If a scaffolding is ok to be performed
             * @return false otherwise
             */
            virtual bool reset(const uint8_t status=0) = 0;
        protected:
            TaskInterface(){}
    };
}
#endif