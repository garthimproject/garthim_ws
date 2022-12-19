#ifndef __NAVIGATION_INTERFACE_GARTHIM_H__
#define __NAVIGATION_INTERFACE_GARTHIM_H__
#include <ros/ros.h>
#include <rl_msgs/RLVariable.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <robot_interface/task_interface.h>
#include <navigation_interface/translator_to_state.h>
#include <navigation_interface/translator_action_to_twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <chrono>
#include <mutex>
#include <string>
namespace navigation_interface {
    /**
     * @brief Sub-class that inherits its methods from TaskInterface and must be
     * loaded using a class loader from ROS.
     * This class serves as an abstraction of a Navigation Task in the context of 
     * a SARSA algorithm. In order to be able to perform an action (a movement step) 
     * this class provides a translator of states, with the mission transforming real-life
     * measures of poses to an abstract convention, and a translator of actions, which
     * is tasked to change velocity command to an abstract nomenclature.
     * 
     */
    class NavigationInterface : public robot_interface::TaskInterface {
        private:
            /**
             * @brief Its mission is to obtain rl_msgs::State from real-time sensorial
             * measures.
             * 
             */
            TranslatorToState translator_to_state_;
            /**
             * @brief This object changes abstract actions received to velocity commands
             * in order to be perform.
             * 
             */
            TranslatorActionToTwist translator_from_action_;
            /**
             * @brief Subscriber for the sensor info. sub_obstacles_ shuts after first
             * batch of messages being received.
             * 
             */
            ros::Subscriber sub_odom_, sub_odom_topic_, sub_scanner_, sub_obstacles_, sub_bumper_, sub_spencer_;
            /**
             * @brief Publisher for velocity commands so the robot can execute them.
             * 
             */
            ros::Publisher pub_cmd_vel_;
            /**
             * @brief Publisher for current state of the robot after an action 
             * has been performed.
             * 
             */
            ros::Publisher pub_current_state_;
            /**
             * @brief Publisher for refreshing/changing targets in a simulator.
             * 
             */
            ros::Publisher pub_target_;
            /**
             * @brief Publisher for current agent pose as it is received.
             * 
             */
            ros::Publisher pub_pose_;
            /**
             * @brief A callback queue to control messages between agent and interface
             * separately from messages between different nodes of the stack
             * 
             */
            ros::CallbackQueue queue_;
            /**
             * @brief cmd_vel_moving_ is the command currently being executed, 
             * cmd_vel_stopping_ is a command to stop the agent.
             * 
             */
            geometry_msgs::Twist cmd_vel_moving_, cmd_vel_stopping_;
            /**
             * @brief Current measure of odometry as ros message.
             * 
             */
            nav_msgs::Odometry current_odometry_;
            nav_msgs::Odometry current_odometry_from_topic_;
            /**
             * @brief A struct containing the state of the bumpers during an execution.
             * 
             */
            CollisionStatus current_collision_status_;
            /**
             * @brief Current laserscam message received.
             * 
             */
            sensor_msgs::LaserScan current_laserscan_;
            /**
             * @brief A message indicating the last persons detected by spencer.
             * 
             */
            spencer_tracking_msgs::TrackedPersons detected_persons_msg_;
            std::vector<double> deriv_dist_ped_;
            int spencer_total_persons_;
            /**
             * @brief Current state to be sent.
             * 
             */
            rl_msgs::State state_;
            /**
             * @brief A parameter indicating the simulated rate at which the interface sends
             * command to the agent.
             * 
             */
            double publish_rate_cmd_;
            /**
             * @brief Name of the model used by the agent.
             * 
             */
            std::string robot_model_name_;
            /**
             * @brief Maximum number of steps that must take place inside an episode.
             * If steps_in_current_episode_ reaches it, it becomes zero and a reset() is called. 
             * 
             */
            int max_steps_per_episode_;
            /**
             * @brief If 1, a stop is issued after sending a command to the robot.
             * If >1, a stop is issued and the interface waits until odometry indicates the 
             * agent has stopped.
             * If 0 or lower, nothing is done.
             * 
             */
            int after_sending_command_;
            /**
             * @brief Variable holding steps performed in current episode.
             * 
             */
            uint64_t steps_in_current_episode_;
            /**
             * @brief If learning is episodic or is performed in only one episode.
             * 
             */
            bool is_episodic_;
            /**
             * @brief If navigation interface is processing messages from the scanner.
             * 
             */
            bool is_laser_equipped_;
            /**
             * @brief Persons detector provided by spencer is used.
             * 
             */
            bool is_spencer_used_;
            bool is_spencer_used_simulated_;
            /**
             * @brief debug params to measure second order moment for distance, its sums and
             * the total of steps respectively.
             * 
             */
            double m2_distance_, sum_distance_, N_;
            /**
             * @brief debug params to measure second order moment for yaw orientation, 
             * its sums and the total of steps respectively.
             * 
             */
            double m2_yaw_, sum_yaw_;
        public:
            NavigationInterface();
            ~NavigationInterface();
            /**
             * @brief From the content of a state object obtains whether agent has
             * reached a goal (2), aborted (1) a goal or continued (0) with an episode.
             * 
             * @param state Contents of a state.
             * @return unsigned char Indicates status of an episode.
             */
            inline virtual unsigned char getType(const rl_msgs::RLVariable& state) {
                if (is_episodic_) {
                    if (translator_to_state_.isFinal(state_.content)) {
                        return 2;
                    } else if (steps_in_current_episode_ == max_steps_per_episode_) {
                        return 1;
                    } 
                }
                return 0;
            }
            /**
             * @brief A method to obtain current state from robot. It delegates to
             * translator_to_state_ object.
             * 
             * @param translation The translation of current real-life measures.
             */
            virtual void getState(rl_msgs::State& translation) override;
            /**
             * @brief Method that sends the real-life commands to the agent in order
             * to be executed. Real-life commands are obtained after translating them
             * from action using translator_from_action_ object.
             * 
             * @param action The action received to be performed.
             * @return double The simulation time that took to send the action.
             */
            virtual double performAction(const rl_msgs::RLVariable& action) override;
            /**
             * @brief Method that issues a reset of the target after an episoded has
             * finished.
             * 
             * @param status How the episode has ended. (success, abort, not-ending)
             * @return true if scaffolding condition is met.
             * @return false otherwise
             */
            virtual bool reset(const uint8_t status=0) override;
            /**
             * @brief Callback to received odometry messages. 
             * Not attached if gazebo is present, as gazebo adds uncertainty to measures.
             * 
             * @param msg odometry message received.
             */
            void receiveOdometry(const nav_msgs::Odometry& msg);
            /**
             * @brief Callback to receive robot model from gazebo, including an accurate 
             * measure of its pose.
             * 
             * @param msg state of all models inside a scene in gazebo.
             */
            void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr msg);
            /**
             * @brief Callback to receive state of obstacles in a scene at the start of 
             * execution. After that, it shuts its publisher down. It generates problems 
             * with mobile obstacles.
             * 
             * @param msg state of all models inside a scene in gazebo.
             */
            void receiveGazeboObstacles(const gazebo_msgs::ModelStates::ConstPtr msg);
            /**
             * @brief Callback to receive messages from laser measures
             * 
             * @param scan Laser scan received.
             */
            void receiveLaser(const sensor_msgs::LaserScan& scan);
            /**
             * @brief Receive bumper info from topics provided by the agent nodes.
             * 
             * @param event Bumper message
             */
            void receiveBumperEvent(const kobuki_msgs::BumperEvent& event);
            /**
             * @brief Callback to receive persons detected by Spencer/Specialised algorithm.
             * 
             * @param msg Message received from spencer.
             */
            void receiveSpencerPersons(const spencer_tracking_msgs::TrackedPersons& msg);
    };
}

#endif