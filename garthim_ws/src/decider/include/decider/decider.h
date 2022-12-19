#ifndef __DECIDER_GARTHIM_H__
#define __DECIDER_GARTHIM_H__
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rl_msgs/DeciderInput.h>
#include <rl_msgs/AgentActionAction.h>
namespace decider {
    /**
     * @brief Generic class for decider entity. This entity is in charge of choosing an action given the values 
     * from value-function for a state and sending the action to the robot in order to be performed.
     * 
     */
    class Decider{
        public:
            virtual ~Decider() {}
            /**
             * @brief Given an input that includes a value-function for current state, get the optimal action.
             * 
             * @param d_input 
             */
            virtual void getAction(const rl_msgs::DeciderInput &d_input) = 0;
        protected:
            /**
             * @brief Client for actionlib AgentAction. This is used to call actions to be performed by the robot.
             * 
             */
            actionlib::SimpleActionClient<rl_msgs::AgentActionAction> ac_;
            rl_msgs::AgentActionGoal goal_;
            /**
             * @brief Construct a new Decider object
             * It also creates the action client for robot_interface and waits for it.
             * 
             */
            Decider() : ac_("perform_action", true) {
                ac_.waitForServer();
                ROS_INFO("[Decider::Decider] Action server perform_action connected");
            }
    };
}
#endif