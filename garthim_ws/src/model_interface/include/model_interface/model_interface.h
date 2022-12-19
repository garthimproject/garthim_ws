#ifndef __MODEL_INTERFACE_GARTHIM_H__
#define __MODEL_INTERFACE_GARTHIM_H__
#include <ros/ros.h>
#include <rl_msgs/GetRandom.h>
#include <rl_msgs/State.h>
#include <robot_interface/task_interface.h>
#include <file_recorder/file_recorder.h>
#include <rl_msgs/UnwrapState.h>
namespace model_interface {
    class ModelInterface : public robot_interface::TaskInterface {
        private:
            ros::NodeHandle nh_;
            rl_msgs::State current_state_;
            std::vector<std::vector<std::vector<double>>> transition_model_;
            bool is_episodic_;
            int64_t spatial_states_;
            file_recorder::FileRecorder file_recorder_;
            int max_steps_per_episode_, current_steps_in_episode_;

            bool is_discrete_;
            bool is_used_spencer_, is_used_laser_, is_using_bumpers_;
            double radius_levels_, orientation_levels_;
            double spencer_orientation_levels_, spencer_orientation_vel_levels_;
            double spencer_orientation_vel_unit_, spencer_depth_levels_;
            double spencer_depth_unit_, spencer_depth_collision_, spencer_total_persons_;
            double scanner_orientation_levels_, scanner_depth_levels_, scanner_depth_unit_;
            double radius_collision_;
            double time_action_levels_;
            ros::ServiceServer server_unwrap_state_;
        public:
            ModelInterface();
            ~ModelInterface();
            virtual void getState(rl_msgs::State& translation) override;
            virtual double performAction(const rl_msgs::RLVariable& action) override;
            virtual bool reset(const uint8_t status = 0) override;
            int getRandom() {
                rl_msgs::GetRandom serv;
                while (!ros::service::call("get_random",serv)) {
                    ros::Duration(0.1).sleep();
                }
                return serv.response.random_number;
            }
            bool isFinal(int state) {
                return is_episodic_ && state % spatial_states_ < orientation_levels_;
            }
            bool unwrapState(rl_msgs::UnwrapStateRequest& req, 
            rl_msgs::UnwrapStateResponse& res);

            int64_t getFirstState(const int64_t last_state);
    };
}
#endif