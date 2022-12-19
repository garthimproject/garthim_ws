#include <pluginlib/class_list_macros.h>
#include <critic/sarsa.h>

PLUGINLIB_EXPORT_CLASS(critic::Sarsa, critic::Critic)
namespace critic {
    Sarsa::Sarsa() : file_recorder_(is_learning_ = !nh_.hasParam("exploitation"))
    {
        ros::NodeHandle nh_private("~");
        steps_ = 1;
        episodes_ = 1;
        max_steps_ = nh_.param("max_steps", 18000);
        max_episodes_ = nh_.param("max_episodes", 500);
        nh_private.param("discounting", discounting_, 0.9);
        pub_sv_function_ = nh_.advertise<rl_msgs::RLVariable>("state_value_function",100,false);
        sub_transition_ = nh_.subscribe("transition_result", 100, &Sarsa::updateValueFunction, this);
        do {
            ros::Duration(1).sleep();
            ROS_INFO("[Sarsa::Sarsa] Waiting for total_states parameter");
        }while (ros::ok() && !nh_.hasParam("total_states")); 
        nh_.param("total_states", total_states_,64);
        ROS_INFO("[Sarsa::Sarsa] total_states: %d",total_states_);
        std::vector<int> action_indexes = {0,1,2,3};
        nh_.getParam("actions_enabled",action_indexes);
        this->total_actions_ = action_indexes.size();
        if (is_learning_) {
            timer_ = nh_.createWallTimer(ros::WallDuration(10), &Sarsa::saveQFileRecorder, this);
        }
        aborted_ = true;
        ROS_INFO("[Sarsa::Sarsa] server exists.");
        service_client_.request.columns.assign(1,0);
        service_client_.request.rows.assign(1,0);
        current_transition_.state.content.as_integer.assign(1,0);
        current_transition_.action.as_integer.assign(1,0);
        current_transition_.next_state.content.as_integer.assign(1,0);
        int scanner_states = 1;
        nh_.param("scanner_states", scanner_states, scanner_states);
        Q_.assign(total_states_*scanner_states, std::vector<double>(total_actions_, 0));
        if (nh_private.hasParam("repeat_distance_states_for_scanner_states") &&
             nh_private.hasParam("url_initial_value_function")) {
            ROS_ERROR("[Sarsa::Sarsa] total_states: %d", total_states_);
            ROS_ERROR("[Sarsa::Sarsa] scanner_states: %d", scanner_states);
            file_recorder_.readValueFunction(Q_,total_states_, scanner_states);
        } else if (nh_private.hasParam("url_initial_value_function")) {
            ROS_INFO("[Sarsa::Sarsa] Load Q without repeating");
            loadValueFunction();
        }
        printQ();
        ROS_INFO("[Sarsa::Sarsa] is_learning? %s", is_learning_? "true":"false");
    }
    Sarsa::~Sarsa() {}
    void Sarsa::updateValueFunction(const rl_msgs::TransitionResult& tr){
        if (steps_ <= max_steps_ && episodes_ <= max_episodes_) {
            const int64_t state = tr.state.content.as_integer[0];
            const int64_t action = tr.action.as_integer[0];
            const int64_t next_state = tr.next_state.content.as_integer[0];
            const double reward = tr.reward, delay=tr.delay, time_elapsed=tr.time_elapsed;
            std::string feedback = tr.next_state.type == 0? "normal":
                tr.next_state.type == 1? "aborted":
                    "final";
            ROS_INFO("[Critic::Sarsa] step:%lu, episode:%lu, s=%ld, a=%ld, S=%ld, r=%f, type: %s",
            steps_-1, episodes_-1, state, action, next_state, reward, feedback.c_str());
            if (!aborted_ && is_learning_) { 
                const int64_t state = current_transition_.next_state.content.as_integer[0];
                const int64_t prev_state = current_transition_.state.content.as_integer[0];
                const int64_t prev_action = current_transition_.action.as_integer[0];
                const double reward = current_transition_.reward;
                service_client_.request.rows[0] = (unsigned short)state;
                service_client_.request.columns[0] = (unsigned short)action;
                while (!ros::service::call("get_suboccurrences", service_client_)) {
                    ros::Duration(0.1).sleep();
                }
                const double occurrences = service_client_.response.data[0];
                /**
                 * const double occurrences = service_client_.response.data[0];
                
                double alpha = occurrences ==0?1:1+0.1/occurrences;
                if (occurrences == 0) {
                    alpha = 1;
                }else if (occurrences > 0 && occurrences < 10) {
                    alpha = 1 - (occurrences-1)*0.1;
                } else {
                    alpha = 1 / occurrences;
                }
                 * 
                 */
                const double alpha = occurrences ==0?1:1/occurrences;
                auto aux = Q_[prev_state][prev_action];
                Q_[prev_state][prev_action] += alpha*(reward+discounting_*Q_[state][action]-Q_[prev_state][prev_action]);
                ROS_WARN("Q[%lu, %lu] = %f + %f*(%f + %f*%f - %f) ",prev_state, prev_action,aux,alpha,reward, discounting_,
                    Q_[state][action],aux);
                ROS_WARN("[Critic::Sarsa] Q[%lu,%lu]: prev=%f -> new=%f", prev_state, prev_action, aux, Q_[prev_state][prev_action]);
                if (prev_state < 8) {
                    steps_ = max_steps_+100;
                    printQ();
                    ROS_INFO("[Critic::Sarsa] step:%lu, episode:%lu, ps=%lu, pa=%lu, pS=%lu, s=%lu, a=%lu, S=%lu, r=%f, type: %s",
            steps_-1, episodes_-1, prev_state, prev_action, state,
                tr.state.content.as_integer[0] ,action,next_state, reward, feedback.c_str());
                }

            }
            current_transition_.state.content.as_integer[0] = state;
            current_transition_.state.type = tr.state.type;
            current_transition_.action.as_integer[0] = action;
            current_transition_.next_state.content.as_integer[0] = next_state;
            current_transition_.next_state.type = tr.next_state.type;
            current_transition_.reward = reward;
            current_transition_.delay = delay;
            current_transition_.time_elapsed = time_elapsed;
            state_value_function_.as_floating = Q_[next_state];
            state_value_function_.as_integer = {next_state};
            pub_sv_function_.publish(state_value_function_);
            steps_++;
            if (current_transition_.next_state.type > 0) {
                episodes_++;
            }
            aborted_ = false;
        } else {
            if (is_learning_) {
                timer_.stop();
                file_recorder_.saveValueFunctionEnd(Q_);
            }
            exit(steps_ == max_steps_+100);
        }
    }
    void Sarsa::loadValueFunction() {
        file_recorder_.readValueFunction(Q_);
    }

    void Sarsa::saveQFileRecorder(const ros::WallTimerEvent& e) {
        file_recorder_.saveValueFunction(Q_,true);
    }
    void Sarsa::printQ() {
        ROS_INFO("[Sarsa::printQ] Q_=");
        std::cout << "  ";
        for (int i = 0; i < total_actions_; i++) {
            std::cout << " " << i;
        }
        std::cout << std::endl;
        for (int i = 0; i < Q_.size(); i++) {
            std::cout << i << ((i < 10)?" ":"") << " ";
            for (int j = 0; j < Q_[i].size(); j++) {
                std::cout << Q_[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
}