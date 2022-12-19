#ifndef __AGENT_ACTION_SERVER_ROBOT_INTERFACE_H__
#define __AGENT_ACTION_SERVER_ROBOT_INTERFACE_H__
#include <actionlib/server/simple_action_server.h>
#include <rl_msgs/AgentActionAction.h>
#include <rl_msgs/GetReward.h>
#include <rl_msgs/GetSubMatrix.h>
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/TransitionResult.h>
#include <pluginlib/class_loader.h>
#include <robot_interface/task_interface.h>
#include <mutex>
#include <memory>
#include <numeric>
#include <std_srvs/Empty.h>
namespace robot_interface {
    typedef actionlib::SimpleActionServer<rl_msgs::AgentActionAction> AgentActionAction;
    /**
     * @brief A Wrapper Class around an actionlib::SimpleActionServer that is tasked 
     * to communicate directly with a real-world interface abstraction to perform tasks and send its results to the other
     * components of the stack, namely Critic and Recorder.
     * 
     */
    class AgentSimpleActionServer {
        private:
        ros::NodeHandle nh_;
        /**
         * @brief An action server object that processes petition to perform tasks (SARSA actions) from action clients.
         * 
         */
        AgentActionAction action_server_;
        /**
         * @brief Publisher for sending messages containing the results from actions after being performed.
         * 
         */
        ros::Publisher pub_transition_result_;
        /**
         * @brief A client to obtain the reward from transitions. (s,a,S)
         * 
         */
        ros::ServiceClient reward_client_;
        /**
         * @brief A server for providing to the nodes of the stack the history of transition-results obtained during a learning experiment.
         * 
         */
        ros::ServiceServer sub_record_server_;
        /**
         * @brief A server for providing to the nodes the occurrences of tuples (s,a) during an execution.
         * 
         */
        ros::ServiceServer sub_occurrences_server_;
        /**
         * @brief A message containing the last feedback sent by the action_server_. (Empty)
         * 
         */
        rl_msgs::AgentActionFeedback feedback_;
        /**
         * @brief A message containing the last result sent by the action_server_. (Empty, provided from topics)
         * 
         */
        rl_msgs::AgentActionResult result_;
        /**
         * @brief Matrix containing the history of transition-results compressed as chars.
         * 
         */
        std::vector<std::vector<unsigned char>> record_;
        /**
         * @brief Matrix containing the occurrences of tuples state-action during an experiment.
         * 
         */
        std::vector<std::vector<long unsigned int>> occurrences_;
        /**
         * @brief Pluginlib class loader to be able to use an interface with the factory abstraction method in ROS architecture.
         * 
         */
        pluginlib::ClassLoader<TaskInterface> ti_loader_;
        /**
         * @brief Shared pointer needed to use class loader object.
         * 
         */
        boost::shared_ptr<TaskInterface> task_interface_;
        /**
         * @brief Flag indicating if is the first state obtained from task_interface_.
         * 
         */
        bool has_first_state_;
        /**
         * @brief Flag indicating if an action is beingn performed currently.
         * 
         */
        bool is_action_being_performed;
        /**
         * @brief Flag indicating if an episode has just ended. In this case, a states are forwarded differently.
         * 
         */
        bool has_episode_ended_;
        /**
         * @brief A method called after every action being performed. It just records the results in the object.
         * 
         */
        void recordAction();
        ros::ServiceServer reset_task_interface_;
        public:
        AgentSimpleActionServer();
        ~AgentSimpleActionServer();
        /**
         * @brief A callback method used by actionlib server. 
         * It performs actions in the real-world by employing a task_interface loaded class.
         * 
         * @param goal The goal of an action. Tipically defines how an action is performed. 
         * It does not have to be a physical goal, the name is just convention.
         */
        void performAction(const rl_msgs::AgentActionGoalConstPtr goal);
        /**
         * @brief Callback assigned to service server. It provides the history of transition-results obtained 
         * during an experiment.
         * 
         * @param req Request indication what portion of the history to be retrieved. (An array of rows and an array of column indexes) 
         * @param res The retrieved history of transition-results.
         * @return true if server call has finished correctly.
         * @return false if somehow server call has failed.
         */
        bool getSubRecord(rl_msgs::GetSubMatrix::Request& req,
            rl_msgs::GetSubMatrix::Response& res);
        /**
         * @brief Callback assigned to service server. It provides the occurrences of pairs state-action during execution.
         * 
         * @param req Request indication what portion of the matrix to be retrieved. (An array of states and an array of action)
         * @param res The retrieved sup-portion of the matrix.
         * @return true if server call has finished correctly.
         * @return false somehow server call has failed.
         */
        bool getSubOccurrences(rl_msgs::GetSubMatrix::Request& req,
            rl_msgs::GetSubMatrix::Response& res);
        
        /**
         * @brief Returns if a flag indicating an action is currently being performed.
         * 
         * @return true if action is being performed.
         * @return false otherwise.
         */
        inline bool isActionBeingPerformed() {
            return is_action_being_performed;
        }

        void init();
        bool reset(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
    };
}
#endif