#ifndef __CRITIC_TABLE_GARTHIM_H__
#define __CRITIC_TABLE_GARTHIM_H__
#include <ros/ros.h>
#include <critic/critic.h>
#include <rl_msgs/RLVariable.h>
#include <rl_msgs/TransitionResult.h>
#include <rl_msgs/GetSubMatrix.h>
#include <file_recorder/file_recorder.h>
namespace critic {
    class Sarsa : public Critic {
        private:
            bool aborted_, is_learning_;
            int total_states_, total_actions_;
            double discounting_;
            std::vector<std::vector<double>> Q_;
            std::string url_initial_value_function_;
            ros::Subscriber sub_transition_;
            ros::Publisher pub_sv_function_;
            rl_msgs::TransitionResult current_transition_;
            rl_msgs::RLVariable state_value_function_;
            ros::NodeHandle nh_;
            rl_msgs::GetSubMatrix service_client_;
            ros::WallTimer timer_;
            uint64_t steps_, max_steps_, max_episodes_;
            uint64_t episodes_;
            file_recorder::FileRecorder file_recorder_;
        public:
            Sarsa();
            ~Sarsa();
            virtual void updateValueFunction(const rl_msgs::TransitionResult& tr) override;
            virtual void loadValueFunction() override;
            void printQ();
            void saveQFileRecorder(const ros::WallTimerEvent& e);
            inline void getQ(std::vector<std::vector<double>>& out) {
                out = Q_;
            }
    };
}
#endif
