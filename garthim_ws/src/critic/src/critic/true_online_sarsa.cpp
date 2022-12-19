#include <pluginlib/class_list_macros.h>
#include <critic/true_online_sarsa.h>

PLUGINLIB_EXPORT_CLASS(critic::TrueOnlineSarsa, critic::Critic)
namespace critic {
    TrueOnlineSarsa::TrueOnlineSarsa() :
     nh_private_("~"),
     file_recorder_(nh_private_.param("increment_file_counter",false)) {
        
        learning_ = !nh_.hasParam("exploitation");
        std::vector<int> actions = {0,1,2,3};
        int total_states, aux_episodes;
        nh_private_.param("max_episodes", aux_episodes, 1000);
        max_episodes_ = aux_episodes;
        nh_.param("total_states", total_states, 64);
        nh_private_.param("step_size", alpha_, 1.0);
        nh_private_.param("lambda", lambda_, 0.9);
        nh_private_.param("discounting", discounting_, 0.9);
        nh_private_.param("threshold", threshold_, 0.01);
        nh_.getParam("actions_enabled", actions);
        Q_.assign( total_states, std::vector<double>(actions.size(),0.0));
        E_.assign( total_states, std::vector<double>(actions.size(),0.0));
        current_episode_ = 0; steps_ = 0;updating_steps_ = 0;
        queue_states_size_ = log(threshold_)/log(discounting_*lambda_);
        pub_sv_function_ = nh_.advertise<rl_msgs::DeciderInput>("state_value_function",100,false);
        sub_transition_ = nh_.subscribe("transition_result", 100, &TrueOnlineSarsa::updateValueFunction, this);
        
        counter_iteration_ = 0;
        nh_private_.param("old_sarsa", old_sarsa_, false);
        prev_v_function_ = 0;
        bool from_pedestrian_learning;
        nh_.param("from_pedestrian_learning", from_pedestrian_learning, false);
        ROS_ERROR("from_pedestrian_learning: %d", int(from_pedestrian_learning));
        if (from_pedestrian_learning &&
            nh_private_.hasParam("repeat_distance_states_for_spencer_states") &&
             nh_private_.hasParam("url_initial_value_function")) {
            ROS_ERROR("from_pedestrian_learning");
            int spencer_states, spencer_states_before;
            nh_.param("spencer_states_before",spencer_states_before, 1);
            nh_.param("spencer_states",spencer_states, 1);
            ROS_INFO("[TrueOnlineSarsa::Sarsa] Load Q repeating: sp=%d, sc=%d",
                total_states/spencer_states_before, spencer_states/spencer_states_before);
            file_recorder_.readValueFunction(Q_,total_states/spencer_states_before, spencer_states/spencer_states_before); 
            
        }else if (nh_private_.hasParam("repeat_distance_states_for_scanner_states") &&
             nh_private_.hasParam("url_initial_value_function")) {
            ROS_ERROR("repeat_distance_states_for_scanner_states");
            int scanner_states;
            nh_.param("scanner_states",scanner_states, 1);
            ROS_INFO("[TrueOnlineSarsa::Sarsa] Load Q repeating: sp=%d, sc=%d",
                total_states/scanner_states, scanner_states);
            file_recorder_.readValueFunction(Q_,total_states/scanner_states, scanner_states);
        } else if (nh_private_.hasParam("repeat_distance_states_for_spencer_states") &&
             nh_private_.hasParam("url_initial_value_function")) {
            ROS_ERROR("repeat_distance_states_for_spencer_states");
            int spencer_states;
            nh_.param("spencer_states",spencer_states, 1);
            ROS_INFO("[TrueOnlineSarsa::Sarsa] Load Q repeating: sp=%d, sc=%d",
                total_states/spencer_states, spencer_states);
            file_recorder_.readValueFunction(Q_,total_states/spencer_states, spencer_states); 
        } else if (nh_private_.hasParam("url_initial_value_function")) {
            ROS_INFO("[TrueOnlineSarsa::Sarsa] Load Q without repeating");
            loadValueFunction();
        }
        nh_private_.param("is_normalized_Q", is_normalized_Q_, false);
        nh_private_.param("is_biased", is_biased_, false);
        nh_private_.param("save_Q", saved_Q_, true);
        normalization_value_Q_ = 100 / (1-discounting_);
        
        url_file_elegibility_ = nh_private_.param("elegibility", std::string("TOSLE"));
        url_file_q_old_= nh_private_.param("q_old", std::string("TOSLQ_old"));
        url_file_queue_states_= nh_private_.param("elegibility", std::string("TOSLQueue_States"));

        printQ();
        ROS_INFO("[TrueOnlineSarsa::Sarsa] is_learning? %s", learning_? "true":"false");
        first_step_in_episode_ = true;
    }
    TrueOnlineSarsa::~TrueOnlineSarsa() { }
    void TrueOnlineSarsa::updateValueFunction(const rl_msgs::TransitionResult& tr) {
        if (current_episode_ < max_episodes_) {
            if (counter_iteration_ > 1 && learning_) {
                int64_t state = current_transition_.state.content.as_integer[0];
                int64_t action = current_transition_.action.as_integer[0];
                int64_t next_state = current_transition_.next_state.content.as_integer[0];
                bool reset_true_online = current_transition_.next_state.type > 0;
                double reward = current_transition_.reward;
                std::string feedback = current_transition_.next_state.type == 0? "normal":
                    current_transition_.next_state.type == 1? "aborted":
                        "final";
                ROS_DEBUG("[Critic::TrueOnlineSarsa] step:%lu, episode:%lu, s=%ld, a=%ld, S=%ld, r=%f, type: %s",
                steps_, current_episode_, state, action, next_state, reward, feedback.c_str());
                int64_t next_action = tr.action.as_integer[0];
                double step_size = getStepSize(state,action);

                

                auto it = std::find(queue_states_.begin(), queue_states_.end(),state);
                if (it != queue_states_.end()) {
                    queue_states_.erase(it);
                    queue_states_.push_back(state);
                } else if (queue_states_.size() < queue_states_size_) {
                    queue_states_.push_back(state);
                } else {
                    queue_states_.pop_front();
                    queue_states_.push_back(state);
                }
                
                if (old_sarsa_) {
                    Q_[state][action] += step_size*(reward+discounting_*Q_[next_state][next_action]-Q_[state][action]);
                } else {
                    updateAccordingToQueue(state, action, reward,
                    next_state, next_action, step_size);
                }

                
                if (saved_Q_ && updating_steps_ % 100 == 0) {
                    saveQFileRecorder();
                    std::vector<double> st;
                    for (auto s:queue_states_) {
                        st.push_back(s);
                    }
                    for (int i = st.size(); i < queue_states_size_; i++) {
                        st.push_back(-1);
                    }
                    file_recorder_.saveGenericVector(this->url_file_queue_states_,st);
                    for (auto e:E_) {
                        file_recorder_.saveGenericVector(url_file_elegibility_,e);
                    }
                    file_recorder_.saveGenericVector(url_file_q_old_, {prev_v_function_});
                }
                if (reset_true_online) {
                    queue_states_ = {};
                    E_.assign(Q_.size(),std::vector<double>(Q_[0].size(),0.0));
                    prev_v_function_ = 0;
                }
                updating_steps_++;
                if(saved_Q_ && tr.next_state.scaffolding > 0) {
                    file_recorder_.saveValueFunctionScaffolding(Q_, true);
                }
            }
            rl_msgs::DeciderInput d_input;
            int64_t next_state;
            if (tr.next_state.type > 0) {
                current_episode_++;
                d_input.is_first_episode_step = true;
                next_state = tr.next_state.content.as_integer[1]; // agent sends reseted state
            } else {
                next_state = tr.next_state.content.as_integer[0];
                d_input.is_first_episode_step = steps_ == 0;
            }
            d_input.state.content.as_integer = {next_state};
            d_input.state.collision = tr.next_state.collision;
            d_input.state.type = tr.next_state.type;
            if (is_biased_) {
                d_input.sv_function = {};
                std::vector<double> biased_Q;
                getBiasedQNextState(next_state, biased_Q);
                if (is_normalized_Q_) {
                    double v = normalization_value_Q_;
                    std::transform(biased_Q.cbegin(), biased_Q.cend(), std::back_inserter(d_input.sv_function),
                        [&v](double q){return 100*q/v;});
                } else {
                    std::transform(biased_Q.cbegin(), biased_Q.cend(), std::back_inserter(d_input.sv_function),
                        [](double q){return q;});
                }
            } else if (is_normalized_Q_) {
                d_input.sv_function = {};
                double v = normalization_value_Q_;
                std::transform(Q_[next_state].cbegin(), Q_[next_state].cend(), std::back_inserter(d_input.sv_function),
                    [&v](double q){return 100*q/v;});
            } else {    
                d_input.sv_function = Q_[next_state];
            }
            d_input.reward = tr.reward;
            pub_sv_function_.publish(d_input);
            steps_++;
            current_transition_ = tr;
            counter_iteration_ += counter_iteration_==2? 0: 1;
        } else {
            if (learning_) {
                file_recorder_.saveValueFunctionEnd(Q_);
            }
            //exit(0);
            std_srvs::Empty msg;
            ros::service::call("reboot_experiment",msg);
            ros::shutdown();
        }
    }
    void TrueOnlineSarsa::updateAccordingToQueue(int64_t state, int64_t action, double reward,
        int64_t next_state, int64_t next_action, double step_size) {
        double delta = reward + discounting_*Q_[next_state][next_action] - Q_[state][action];
        double diff_q = Q_[state][action] - prev_v_function_;
        prev_v_function_ = Q_[next_state][next_action];
        E_[state][action] = lambda_*discounting_*(1 - step_size)*E_[state][action] + 1;
        Q_[state][action] += step_size*(delta+diff_q)*E_[state][action]-step_size*diff_q; 
        for (auto it = queue_states_.cbegin(); it != queue_states_.cend(); it++) {
            int64_t i = *it;
            for(int64_t j = 0; j < Q_[i].size(); j++) {
                bool not_first = i != state || j != action;          
                if (not_first&&E_[i][j] > threshold_) {
                    E_[i][j] *= lambda_*discounting_; 
                    Q_[i][j] += step_size*(delta+diff_q)*E_[i][j];
                } else if (not_first) {
                    E_[i][j] = 0;
                }
            }
        }
    }
    double TrueOnlineSarsa::getStepSize(int64_t state, int64_t action) {
        
        /*rl_msgs::GetSubMatrix service_client;
        service_client.request.columns.assign(1,0);
        service_client.request.rows.assign(1,0);
        service_client.request.rows[0] = state;
        service_client.request.columns[0] = action;
        while (!ros::service::call("get_suboccurrences", service_client)) {
            ros::Duration(0.1).sleep();
        }
        double occurrences = service_client.response.data[0];
        return occurrences == 0? alpha_: alpha_/occurrences;*/
        return alpha_;
    }
    void TrueOnlineSarsa::saveQFileRecorder() {
        file_recorder_.saveValueFunction(Q_,true);
    }
    void TrueOnlineSarsa::loadValueFunction() {
        file_recorder_.readValueFunction(Q_);
    }
    void TrueOnlineSarsa::printQ() {
        int start_row = 0;
        if (Q_.size() > 128) {
            start_row = Q_.size() - 128;
            ROS_WARN("[TrueOnlineSarsa::printQ] Value-Function has too many states (>128). Will show last 128 rows.");
        }
        ROS_INFO("[TrueOnlineSarsa::printQ] Q_=");
        int num_spaces = std::to_string(start_row).size();
        for (int i = 0; i < num_spaces; i++) {
            std::cout << " ";
        }
        for (int i = 0; i < Q_[0].size(); i++) {
            std::cout << " " << i;
        }
        std::cout << std::endl;
        for (int i = start_row; i < Q_.size(); i++) {
            std::cout << i << ((i < 10)?" ":"") << " ";
            for (int j = 0; j < Q_[i].size(); j++) {
                std::cout << Q_[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void TrueOnlineSarsa::getBiasedQNextState(const int64_t next_state, std::vector<double>& out) {
        std::vector<int> unwrapped_state;
        rl_msgs::UnwrapState service_client;
        service_client.request.state = next_state;
        while (!ros::service::call("unwrap_state", service_client)) {
            ros::Duration(0.1).sleep();
            ROS_DEBUG("Waiting for translator to unwrap");
        }
        std::stringstream ss;

        ROS_DEBUG("[TrueOnlineSarsa::wrapState] original_state: %ld", next_state);
        ROS_DEBUG("START");
        for (int64_t a = 0; a < Q_[next_state].size(); a++) {
            double qbiased=0;
            ROS_DEBUG("ACTION: %ld", a);
            for (int i = 0; i<service_client.response.size_states.size(); i++) {
                double inner_mean=0;
                for (int j = 0; j < service_client.response.size_states[i]; j++) {
                    int64_t new_state;
                    if (service_client.response.unwrapped_state[i] != j) {
                        new_state = wrapState(j,i, service_client.response.unwrapped_state, service_client.response.size_states);
                    } else {
                        new_state = next_state;
                    }

                    ROS_DEBUG("[TrueOnlineSarsa::wrapState] wrapped_state: %ld", new_state);
                    inner_mean+=Q_[new_state][a];
                }
                ROS_DEBUG("NEXT");
                inner_mean /= service_client.response.size_states[i];
                qbiased += inner_mean;
            }
            ROS_DEBUG("ACTION END");
            qbiased /= service_client.response.size_states.size();
            ss << "bias: " << qbiased << " ";
            out.push_back(Q_[next_state][a]+qbiased);
            ss << "q+bias: " << Q_[next_state][a]+qbiased << std::endl;
        }
        std::string str = ss.str();
        ROS_DEBUG("[TrueOnlineSarsa::getBiasedQNextState] %s",str.c_str());
    }

    int64_t TrueOnlineSarsa::wrapState(const int new_value, const int id_state, const std::vector<int64_t>& unwrapped_state, const std::vector<int64_t>& sizes_state) {
        int64_t state = 0, total_state=1;
        std::stringstream ss, ss2;
        for (int i = unwrapped_state.size()-1; i>=0; i--) {
            ss << (id_state==i?new_value:unwrapped_state[i]) << ", ";//size: " << sizes_state[i]<< ", ";
            ss2 << sizes_state[i] << ", ";
            state += total_state*(id_state==i?new_value:unwrapped_state[i]);
            total_state *= sizes_state[i];
        }
        ROS_DEBUG("[wrapState] \n[%s]\n[%s]", ss.str().c_str(), ss2.str().c_str());
        return state;
    }
}
