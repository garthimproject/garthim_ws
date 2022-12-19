#include <pluginlib/class_list_macros.h>
#include <model_interface/model_interface.h>
PLUGINLIB_EXPORT_CLASS(model_interface::ModelInterface,robot_interface::TaskInterface)
namespace model_interface {
    ModelInterface::ModelInterface() : 
        nh_("~"),
        file_recorder_(nh_.param("increment_file_counter",false)) 
        {
        ros::NodeHandle nh_global;
        nh_.param("episodic", is_episodic_,false);
        if (is_episodic_) {
            ROS_INFO("[ModelInterface::ModelInterface] Episodic");
            nh_.param("max_steps_per_episode", max_steps_per_episode_, 250);
            current_steps_in_episode_ = 0;
        }
        file_recorder_.readModel(transition_model_, 0);
        if (is_used_spencer_ = nh_.hasParam("spencer_topic")) {
            nh_.param("spencer_orientation_levels", spencer_orientation_levels_, 3.0);
            nh_.param("spencer_orientation_vel_levels", spencer_orientation_vel_levels_, 2.0);
            nh_.param("spencer_orientation_vel_unit", spencer_orientation_vel_unit_, 0.30);
            nh_.param("spencer_depth_levels", spencer_depth_levels_, 3.0);
            nh_.param("spencer_depth_unit", spencer_depth_unit_, 0.30);
            nh_.param("spencer_depth_collision", spencer_depth_collision_, 0.50);
            nh_.param("spencer_total_persons", spencer_total_persons_, 2.0);
            
            scanner_depth_levels_=1;
            scanner_orientation_levels_=0;
        }
        if (is_used_laser_ = nh_.hasParam("laser_topic")) {
            nh_.param("scanner_orientation_levels",scanner_orientation_levels_, 4.0);
            nh_.param("scanner_depth_levels",scanner_depth_levels_, 3.0);
        }
        nh_.param("is_using_kobuki_bumpers", is_using_bumpers_, false);

        nh_global.param("time_actions_states", time_action_levels_, 1.0);
        if (is_discrete_ = nh_.hasParam("discrete")) {
            nh_.param("radius_levels", this->radius_levels_, 8.0);
            nh_.param("orientation_levels",this->orientation_levels_, 8.0);
            spatial_states_= orientation_levels_*radius_levels_;
        }
        server_unwrap_state_ = nh_global.advertiseService("unwrap_state", &ModelInterface::unwrapState, this);
        int state;
        nh_.param("initial_state", state, 2279);
        current_state_.content.as_integer.assign(1,getFirstState(state));
    }
    ModelInterface::~ModelInterface() {}
    void ModelInterface::getState(rl_msgs::State& translation) {
        translation.content.as_integer[0] = current_state_.content.as_integer[0];
        translation.type = current_state_.type;
        translation.collision = current_state_.collision;
        if (is_episodic_ && current_state_.type > 0) {
            reset(current_state_.type);
            if (translation.content.as_integer.size() < 2) {
                translation.content.as_integer.emplace_back();
            }
            translation.content.as_integer[1] = current_state_.content.as_integer[0];
        }
    }

    double ModelInterface::performAction(const rl_msgs::RLVariable& action) {
        int R = getRandom();
        double r = R%100;
        r /= 100;
        auto s = current_state_.content.as_integer[0],a=action.as_integer[0], S = current_state_.content.as_integer[0];
        double cumm_prob = 0;
        bool selected = false;
        int i = 0;
        while(i < transition_model_[s][a].size() && !selected) {
            if (cumm_prob <= r && r < cumm_prob + transition_model_[s][a][i]) {S = i; selected = true;}
            cumm_prob += transition_model_[s][a][i++];
        }
        //ROS_INFO("[ModelInterface::performAction] current_state saving");
        current_state_.content.as_integer[0] = selected? S : R%transition_model_[s][a].size();
        rl_msgs::UnwrapStateRequest req;
        req.state = current_state_.content.as_integer[0];
        rl_msgs::UnwrapStateResponse res;
        unwrapState(req,res);
        i=0;
        bool collision = false;
        while(i<int(scanner_orientation_levels_) && !collision) {
            collision = res.unwrapped_state[i++] == 0;
        }
        current_state_.collision = collision;
        bool is_final = is_episodic_ && current_state_.content.as_integer[0] % spatial_states_ < orientation_levels_;
        bool is_aborted = is_episodic_ && current_steps_in_episode_+1 == max_steps_per_episode_;
        if (is_final) {
            current_steps_in_episode_ = 0;
            current_state_.type = 2;
        } else if (is_aborted) {
            current_steps_in_episode_ = 0;
            current_state_.type = 1;
        } else {
            current_steps_in_episode_++;
            current_state_.type = 0;
        }
        //ROS_INFO("[ModelInterface::performAction] current_state saving END");
        return 0.0;
    }

    bool ModelInterface::reset(const uint8_t status /* = 0*/) {
        if (status == 1) {
            ROS_DEBUG("Episode Aborted");
        } else if (status == 2) {
            ROS_DEBUG("Goal Reached");
        }
        current_state_.content.as_integer[0] = getFirstState(current_state_.content.as_integer[0]);
        //current_state_.type = 0;
        return false;
    }

    bool ModelInterface::unwrapState(rl_msgs::UnwrapStateRequest& req, 
            rl_msgs::UnwrapStateResponse& res) {
        int64_t state = req.state;
        int64_t ol = orientation_levels_, rl = radius_levels_, al=time_action_levels_;
        std::stringstream ss;
        if (is_used_spencer_) {
            int64_t digit_r, digit_th, digit_v;
            //res.size_states.push_back(spencer_orientation_levels_);
            ss << std::endl <<"state " << state << std::endl;
            for (int i = spencer_total_persons_-1; i >= 0; i--) {
                int64_t base = pow(scanner_depth_levels_,scanner_orientation_levels_)*ol*rl*al;
                base *= pow(spencer_orientation_vel_levels_*spencer_orientation_levels_*spencer_depth_levels_,i)
                        *spencer_orientation_levels_*spencer_depth_levels_;
                digit_v = state / base;
                res.unwrapped_state.push_back(digit_v);
                //res.size_states.push_back(spencer_depth_levels_);
                res.size_states.push_back(spencer_orientation_vel_levels_);
                state -= base*digit_v;
                ss << "digit " << digit_v << ", base " << base <<", state " << state << std::endl;

                base /= spencer_depth_levels_;
                digit_r = state / base;
                res.unwrapped_state.push_back(digit_r);
                res.size_states.push_back(spencer_orientation_levels_);
                state -= base*digit_r;
                ss << "digit " << digit_r << ", base " << base <<", state " << state << std::endl;

                base /= spencer_orientation_levels_;
                digit_th = state / base;
                res.unwrapped_state.push_back(digit_th);
                //res.size_states.push_back(spencer_orientation_levels_);
                res.size_states.push_back(spencer_depth_levels_);
                state -= base*digit_th;
                ss << "digit " << digit_th << ", base " << base <<", state " << state << std::endl;
            }
            /*
            int64_t base = pow(scanner_depth_levels_,scanner_orientation_levels_)*ol*rl*al;
            int64_t digit = state/base;
            res.unwrapped_state.push_back(digit);
            state -= base*digit;

            ss << "digit " << digit << ", base " << base <<", state " << state << std::endl; */
        } 
        if (is_used_laser_) {
            int64_t digit;
            int init_loop = scanner_orientation_levels_ - 1;
            if (is_using_bumpers_) {
                res.size_states.push_back(4);
                init_loop = scanner_orientation_levels_;
            } else {
                res.size_states.push_back(scanner_depth_levels_);
            }
            ss << "state " << state << std::endl;
            for (int i = init_loop; i > 0; i--) {
                int64_t base = pow(scanner_depth_levels_,i)*ol*rl*al;
                digit = state / base;
                res.unwrapped_state.push_back(digit);
                res.size_states.push_back(scanner_depth_levels_);
                state -= base*digit;

                ss << "digit " << digit << ", base " << base <<", state " << state << std::endl;
            }
            digit = state/ol/rl/al;
            res.unwrapped_state.push_back(digit);
            state -= ol*rl*al*digit;

            ss << "digit " << digit << ", base " << ol*rl <<", state " << state << std::endl;
        }
        if (time_action_levels_ > 1.0) {
            int64_t base = ol*rl;
            int64_t digit = state/base;
            res.unwrapped_state.push_back(digit);
            res.size_states.push_back(al);
            state -= ol*rl*digit;
        }
        res.unwrapped_state.push_back(state/ol);
        res.unwrapped_state.push_back(state%ol);
        res.size_states.push_back(rl);
        res.size_states.push_back(ol);
        ss <<  "state1 " << state/ol <<", state2 " << state%ol;
        std::string str = ss.str();
        ROS_DEBUG("%s", str.c_str());

        return true;
    }

    int64_t ModelInterface::getFirstState(int64_t prev_state) {
        int r = getRandom()%int(orientation_levels_);
        int64_t state = r + 2*orientation_levels_;
        if (time_action_levels_>1){
            state+=int(time_action_levels_)/2*orientation_levels_*radius_levels_;
        }
        if (is_used_laser_) {
            rl_msgs::UnwrapStateRequest req;
            req.state = prev_state;
            rl_msgs::UnwrapStateResponse res;
            unwrapState(req,res);
            int64_t base = 1;
            for (int i = 0; i < scanner_orientation_levels_; i++) {
                state+= res.unwrapped_state[i]*base*time_action_levels_*orientation_levels_*radius_levels_;
                base *= scanner_depth_levels_;
            }
        }
        return state;
    }
}