#ifndef __TRANSLATOR_ACTION_TO_TWIST_NAVIGATION_INTERFACE_H__
#define __TRANSLATOR_ACTION_TO_TWIST_NAVIGATION_INTERFACE_H__
#include <ros/ros.h>
#include <rl_msgs/RLVariable.h>
#include <geometry_msgs/Twist.h>
namespace navigation_interface {
    /**
     * @brief A class tasked to translate abstract actions message to velocity commands as Twist messages.
     * 
     */
    class TranslatorActionToTwist{
        private:
            /**
             * @brief Whether actions received are discrete or continuous ones.
             * 
             */
            bool is_discrete_;
            /**
             * @brief Number of actions choosable.
             * 
             */
            int total_actions_;
            /**
             * @brief Module of linear velocity for building twist messages.
             * 
             */
            double linear_velocity_module_;
            /**
             * @brief Module of angular velocity for building twist messages.
             * 
             */
            double angular_velocity_module_;
            /**
             * @brief Time of each action that is enables. (Default t = [1])
             * 
             */
            std::vector<double> action_durations_;
            /**
             * @brief Indexes of each action implemented.
             * 
             */
            std::vector<int> action_indexes_;
            /**
             * @brief Index inside action_durations_ of current action.
             * 
             */
            int id_current_action_duration_;
            /**
             * @brief A parameter used to determined whether a velocity command implies movement.
             * 
             */
            constexpr static double VELOCITY_COMMAND_EPSILON_ = 1E-2;
            
        public:
            TranslatorActionToTwist();
            ~TranslatorActionToTwist();
            /**
             * @brief This method transforms a rl_msgs::RLVariable message to a geometry_msgs::Twist message
             * in order to be send to the agent. It delegates to translateDiscrete() or translateContinuous() 
             * depending on whether discrete mode is set or not.
             * 
             * @param input An abstract action received from rl stack.
             * @param translation A translation message.
             * @return true if an action implies movement.
             * @return false otherwise.
             */
            inline bool translate(const rl_msgs::RLVariable& input,
             geometry_msgs::Twist& translation) {
                return is_discrete_? translateDiscrete(input, translation)
                    : translateContinuous(input, translation);
            }

            inline double getLinearDistanceFromVelocityAndCurrentDuration() const {
                return linear_velocity_module_*action_durations_[id_current_action_duration_] + 0.1;
            }
            inline double getAngularDistanceFromVelocityAndCurrentDuration() const {
                return angular_velocity_module_*action_durations_[id_current_action_duration_];
            }
            /**
             * @brief Get the duration for current action being executed.
             * 
             * @return double indicating the duration of current action.
             */
            inline double getDurationForAction() const {
                return action_durations_[id_current_action_duration_];
            }
            /**
             * @brief Get the index of current action being executed.
             * 
             * @return int. index of current action being executed.
             */
            inline int getIdCurrentActionDuration() const {
                return id_current_action_duration_;
            }
            /**
             * @brief Get total number of action durations that are choosable.
             * 
             * @return std::size_t Number of duractions that this translator supports.
             */
            inline std::size_t getTotalActionDurations() const {
                return action_durations_.size();
            }

            /**
             * @brief This method interpretes a rl_msgs::RLVariable message in geometry_msgs::Twist message
             * in order to be send to the agent.
             * 
             * @param input Action message from rl-stack. In discrete form.
             * @param translation Twist message result of the translation. 
             * @return true if action implies movement.
             * @return false otherwise.
             */
            bool translateDiscrete(const rl_msgs::RLVariable& input,
             geometry_msgs::Twist& translation);
             /**
             * @brief (Not implemented yet) This method interpretes a rl_msgs::RLVariable message in geometry_msgs::Twist message
             * in order to be send to the agent.
             * 
             * @param input Action message from rl-stack. In continuous form.
             * @param translation Twist message result of the translation. 
             * @return true if action implies movement.
             * @return false otherwise.
             */
            bool translateContinuous(const rl_msgs::RLVariable& input,
             geometry_msgs::Twist& translation);
             /**
             * @brief This method checks if a velocity command
             *  is really close to being nil.
             * 
             * @param <const geometry_msgs::Twist&> cmd_vel 
             * @return true if module of every vector is less than VELOCITY_COMMAND_EPSILON_
             * @return false if any of the forementioned does not apply
             */
            inline static bool isActionFinished(const geometry_msgs::Twist& cmd_vel) {
                return abs(cmd_vel.linear.x) < VELOCITY_COMMAND_EPSILON_
                    && abs(cmd_vel.linear.y) < VELOCITY_COMMAND_EPSILON_
                    && abs(cmd_vel.linear.z) < VELOCITY_COMMAND_EPSILON_
                    && abs(cmd_vel.angular.x) < VELOCITY_COMMAND_EPSILON_
                    && abs(cmd_vel.angular.y) < VELOCITY_COMMAND_EPSILON_
                    && abs(cmd_vel.angular.z) < VELOCITY_COMMAND_EPSILON_;
            }
    };
}
#endif