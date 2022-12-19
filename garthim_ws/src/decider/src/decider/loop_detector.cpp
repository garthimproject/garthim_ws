#include <decider/loop_detector.h>

namespace decider {
    LoopDetector::LoopDetector() : LoopDetector(0,0) {
    }
    LoopDetector::LoopDetector(double temperature, double lambda) : file_recorder_(true) {
        ros::NodeHandle nh, nh_private("~");
        is_first_reward_ = true;
        is_first_call_ = true;
        current_temperature_ = temperature;
        initial_temperature_ = temperature;
        lambda_ = lambda;
        int total_states;
        nh.param("total_states",total_states,64);
        min_size_ = 4;
        max_size_ = total_states / 4;
        steps_ = 1;
        filename_queue_states_ += nh_private.param("queue_states", std::string("DLEQueueStates"));
        filename_reward_states_ += nh_private.param("queue_rewards", std::string("DLEQueueRewards"));
        filename_steps_ += nh_private.param("steps_temperature", std::string("DLEStepsTemperature"));
        filename_q_s_ += nh_private.param("q_s", std::string("DLEQS"));
        nh_private.param("is_decreasing", is_decreasing_, false);
    }
    LoopDetector::~LoopDetector() {}

    double LoopDetector::detectLoop(const rl_msgs::DeciderInput &d_input) {
        if (!is_first_call_) {
            if (d_input.state.type > 0) {
                queue_states_ = std::list<int64_t>();
                queue_rewards_ = std::list<double>();
                set_states_ = std::unordered_set<int64_t>();
                current_temperature_ = initial_temperature_;
                is_first_reward_ = true;
            } else if (queue_states_.size() >= max_size_) {
                int64_t front_state = queue_states_.front();
                double front_reward = queue_rewards_.front();
                reward_avg_ = (reward_avg_*queue_rewards_.size() - front_reward)/(queue_rewards_.size()-1);
                queue_states_.pop_front();
                queue_rewards_.pop_front();
                auto it = std::find(queue_states_.begin(), queue_states_.end(),front_state);
                if (it == queue_states_.end()) {
                    set_states_.erase(front_state);
                }
            }
            int64_t state = d_input.state.content.as_integer[0];
            queue_states_.push_back(state);
            queue_rewards_.push_back(d_input.reward);
            auto pair = set_states_.insert(state);
            reward_avg_ = is_first_reward_? d_input.reward
                    : (reward_avg_*(queue_rewards_.size()-1)+d_input.reward)/queue_rewards_.size();
            is_first_reward_ = false;
            if (queue_states_.size() >= min_size_) {
                bool repeated = !pair.second;
                double redundancy = double(queue_states_.size())/set_states_.size();
                if (repeated && reward_avg_ < 0 && redundancy > 2) {
                    current_temperature_ += 0.25*redundancy;
                    std::vector<double> v_steps = {double(steps_),current_temperature_, redundancy};
                    file_recorder_.saveGenericVector(filename_steps_, v_steps);
                } else {
                    current_temperature_ = initial_temperature_;
                    if (is_decreasing_) {
                        this->initial_temperature_ *= this->lambda_;
                    }
                }
                
            }
            std::vector<double> v_q_states, v_q_rewards;
            for (auto s: queue_states_) {
                v_q_states.push_back(s);
            }
            for (auto r: queue_rewards_) {
                v_q_rewards.push_back(r);
            }
            file_recorder_.saveGenericVector(filename_queue_states_, v_q_states);
            file_recorder_.saveGenericVector(filename_reward_states_, v_q_rewards);
            file_recorder_.saveGenericVector(filename_q_s_, d_input.sv_function);
            steps_++;
           
        }
        is_first_call_ = false;
        return current_temperature_;
    }

}