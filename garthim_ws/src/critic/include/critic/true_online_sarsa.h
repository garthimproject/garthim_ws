#ifndef __CRITIC_TRUE_ONLINE_SARSA_GARTHIM_H__
#define __CRITIC_TRUE_ONLINE_SARSA_GARTHIM_H__
#include <critic/critic.h>
#include <ros/ros.h>
#include <rl_msgs/GetSubMatrix.h>
#include <unordered_set>
#include <file_recorder/file_recorder.h>
#include <rl_msgs/DeciderInput.h>
#include <rl_msgs/UnwrapState.h>
#include <std_srvs/Empty.h>
namespace critic {
    /**
     * @brief Implementation of Critic interface based on TOSL.
     * ([1] - Angel Martínez-Tenor, Juan Antonio Fernández-Madrigal, Ana Cruz-Martín and Javier González-Jiménez, 
     * Towards a common implementation of reinforcement learning for multiple robotics tasks. February 2017.
     * DOI.  10.1016/j.eswa.2017.11.011)
     * 
     */
    class TrueOnlineSarsa : public Critic {
        private:
            /**
             * @brief Name of the file for storing Elegibility Traces for each meaningful step.
             * 
             */
            std::string url_file_elegibility_;
            /**
             * @brief (Debug purposes) Name of the file for storing q_old for each meaningful step. (Debug purposes)
             * 
             */
            std::string url_file_q_old_;
            /**
             * @brief Name of the file for storing variable queue_states_ for each meaningful step.
             * 
             */
            std::string url_file_queue_states_;
            /**
             * @brief Matrix used to store current Value-Function in a tabular form.
             * 
             */
            std::vector<std::vector<double>> Q_;
            /**
             * @brief Matrix used to store current Elegibility Traces.
             * 
             */
            std::vector<std::vector<double>> E_;
            /**
             * @brief A limited queue used to store the different states visited during an episode that are relevant for the 
             * learning process.
             * 
             */
            std::list<int64_t> queue_states_;
            /**
             * @brief Limit of the queue_states_ list.
             * 
             */
            int queue_states_size_;
            /**
             * @brief Parameter used to compute the actual step size of the learning process. (learning rate)
             * 
             */
            double alpha_;
            /**
             * @brief Lambda parameter used in SARSA(LAMBDA) family Algorithms.
             * 
             */
            double lambda_;
            /**
             * @brief Often seeing in the documentation as GAMMA, this parameter is used in RL algoritms to reduce the impact 
             * of the v(next_state) values in the learning process. v being the value-function.
             * 
             */
            double discounting_;
            /**
             * @brief This threshold is used to shut-down to 0 the values of Elegibility Trace for states that were visited
             *  with longevity in an episode.
             * 
             */
            double threshold_;
            /**
             * @brief This parameter is refered as q_old or Qold in the literature. It is the value of v(next_state) from 
             * the previous iteration. v(next_state) is the value of value-function for the next_state computed in a SARSA step.
             */
            double prev_v_function_;
            /**
             * @brief This value is used to normalized v(s)
             * 
             */
            double normalization_value_Q_;
            /**
             * @brief current_episode_: variable that stores an index of the current episode.
             * 
             */
            uint64_t current_episode_;
            /**
             * @brief max_episodes_: variable that stores the maxima of episodes that can take place in a learning process.
             * 
             */
            uint64_t max_episodes_;
            /**
             * @brief steps_: The total amount of updateValueFunction() calls received.
             * 
             */
            uint64_t steps_;
            bool first_step_in_episode_;
            /**
             * @brief updating_steps_: Number of actual updates of Q_.
             * Value used to know when to output debug files (tipically, every 100 steps we output Q_ and the other params)
             * 
             */
            uint64_t updating_steps_;
            /**
             * @brief Current transition that is going to be used to update value-function. It contains the previous action applied, which is 
             * the current action for which the value-function is going to be learnt.
             */
            rl_msgs::TransitionResult current_transition_;
            /**
             * @brief NodeHandle using a global namespace for subscribing, advertising and loading global params.
             * 
             */
            ros::NodeHandle nh_;
            /**
             * @brief NodeHandle using a private namespace for loading params.
             * 
             */
            ros::NodeHandle nh_private_;
            /**
             * @brief Subscriber for receiving rl_msgs::TransitionResult messages. Callback is: updateValueFunction()
             * 
             */
            ros::Subscriber sub_transition_;
            /**
             * @brief Publisher for sending rl_msgs::DeciderInput messages.
             * 
             */
            ros::Publisher pub_sv_function_;
            /**
             * @brief Tipically in a SARSA algorithm, the first learning step is performed when a second action is performed.
             * This flag is employed to skip learning for the first action when a second one is yet to be performed.
             * 
             */
            char counter_iteration_;
            /**
             * @brief This flag enables learning at all. When set to false, value-function is not updated.
             * 
             */
            bool learning_;
            /**
             * @brief Flag used to enable TOSL when false or SARSA when true (skipping queue and eligibility trace usage). 
             * 
             */
            bool old_sarsa_;
            /**
             * @brief When this flag is set to true, values from value-function are normalized by normalization_value_Q_
             * before sending them to Decider.
             * Values are not stored normalized, normalization only applies to DeciderInput values.
             * 
             */
            bool is_normalized_Q_;
            /**
             * @brief This flag enables whether a bias is applied to values to be sent to Decider.
             * Values are not stored with bias, bias is only applied for outputs of updateValueFunction() (Messages for Decider).
             * 
             */
            bool is_biased_;
            /**
             * @brief This flag enables storing Q and related values in disk.
             * 
             */
            bool saved_Q_;
            /**
             * @brief Object that wraps the methods to store in files the learning parameters.
             * 
             */
            file_recorder::FileRecorder file_recorder_;
        public:
            TrueOnlineSarsa();
            ~TrueOnlineSarsa();
            /**
             * @brief Callback for publisher. It implements a learning step for either TOSL or SARSA. 
             * Each step may perform writing-file operations. It enables normalization and biased learning as is described in [1].
             * 
             * @param tr <const rl_msgs::TransitionResult&> The current Transition message received from robot-interface. 
             * Tipically this is the output that is produced after a step is performed in the real world and the reward for said step is received.
             */
            virtual void updateValueFunction(const rl_msgs::TransitionResult& tr) override;
            /**
             * @brief This method performs TOSL step for states inside queue_states_.
             * 
             * @param state Current state.
             * @param action Current action to be learnt.
             * @param reward Reward for s,a
             * @param next_state Next state.
             * @param next_action Next action.
             * @param alpha The learning rate for said step.
             */
            void updateAccordingToQueue(int64_t state, int64_t action, double reward,
                int64_t next_state, int64_t next_action, double alpha);
            /**
             * @brief Get the value of the learning rate for current state and action.
             * 
             * @param state Current state.
             * @param action Current action.
             * @return double Step size.
             */
            double getStepSize(int64_t state, int64_t action);
            /**
             * @brief Method called when learning has finished. It stores current value-function in a file with a distinct name.
             * 
             */
            void saveQFileRecorder();
            /**
             * @brief Method to load a previously learnt Value Function in the beginning of the learning process.
             * 
             */
            virtual void loadValueFunction() override;

            /**
             * @brief Method used to console output value function in the beginning of execution.
             * 
             */
            void printQ();
            /**
             * @brief Compute value-function with bias for next_state.
             * 
             * @param next_state The next state used by decider to choose an action.
             * @param out The value of Q(next_state) after bias is applied.
             */
            void getBiasedQNextState(const int64_t next_state,std::vector<double>& out);
            /**
             * @brief A Method that creates a state knowing each variable that composes it. An arbitrary value is used to set a variable and the rest is used from next state.
             * 
             * @param new_value The value of the fixed variable
             * @param id_state An index to identify the variable that is fixed to create the new state inside unwrapped_state parameter.
             * @param unwrapped_state Next state unwrapped.
             * @param sizes_state The number of possible states for each variable unwraped in unwrapped_state
             * @return int64_t. A new state generated.
             */
            static int64_t wrapState(const int new_value, const int id_state, const std::vector<int64_t>& unwrapped_state, const std::vector<int64_t>& sizes_state);
    };
}
#endif