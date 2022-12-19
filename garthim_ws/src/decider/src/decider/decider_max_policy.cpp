#include <pluginlib/class_list_macros.h>
#include <decider/decider_max_policy.h>
PLUGINLIB_EXPORT_CLASS(decider::DeciderMaxPolicy, decider::Decider)

namespace decider {
    DeciderMaxPolicy::DeciderMaxPolicy() : Decider() {
        ros::NodeHandle nh,nh_private("~");
        //sub_sv_function_ = nh.subscribe("state_value_function", 100, &DeciderMaxPolicy::getAction, this);
        goal_.action.as_integer.assign(1,0);
        nh_private.param("loop_detection_enabled", loop_detection_, false);
        if (loop_detection_) {
            loop_detector_ = std::unique_ptr<LoopDetector>(new LoopDetector);
        }
        nh_private.param("threshold_maxima", thresh_, 0.00001);
        srand(0);
    }
    DeciderMaxPolicy::~DeciderMaxPolicy() {}
    void DeciderMaxPolicy::getAction(const rl_msgs::DeciderInput &d_input) {
        if (loop_detection_) {
            loop_detector_->detectLoop(d_input);
        }
        const auto sv_f = d_input.sv_function;
        double max;
        std::vector<size_t> max_ids = {};
        if (!d_input.sv_function.empty()) {
            max = d_input.sv_function[0];
            max_ids.push_back(0);
            for (size_t i = 1; i < d_input.sv_function.size(); i++) {
                double curr_val = d_input.sv_function[i];
                double diff = abs(max - curr_val);
                if (diff < thresh_) {
                    max_ids.push_back(i);
                } else if (max < curr_val) {
                    max = curr_val;
                    max_ids = {i};
                }
            }
            goal_.action.as_integer[0] = max_ids[rand()%max_ids.size()];
            ac_.sendGoal(goal_);
        }
    }
}