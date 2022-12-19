#include <pluginlib/class_list_macros.h>
#include <decider/decider_egreedy.h>
PLUGINLIB_EXPORT_CLASS(decider::DeciderEGreedy, decider::Decider)

namespace decider {
    DeciderEGreedy::DeciderEGreedy() : Decider() {
        ros::NodeHandle nh,nh_private("~");
        nh_private.param("epsilon", epsilon_, 0.3);
        //sub_sv_function_ = nh.subscribe("state_value_function", 100, &DeciderEGreedy::getAction, this);
        goal_.action.as_integer.assign(1,0);
        ROS_WARN("[DeciderEGreedy::DeciderEGreedy] Decider egreedy initialized");
    }
    DeciderEGreedy::~DeciderEGreedy() {}
    void DeciderEGreedy::getAction(const rl_msgs::DeciderInput &d_input) {
        const auto sv_f = d_input.sv_function;
        uint64_t a = ran_gen_.uniform01() < 1-epsilon_ ?
            std::distance(sv_f.cbegin(), std::max_element(sv_f.cbegin(), sv_f.cend()))
            : ran_gen_.uniformInteger(0, sv_f.size() - 1);
        
        goal_.action.as_integer[0] = a;
        ac_.sendGoal(goal_);
    }
}