#include <pluginlib/class_list_macros.h>
#include <decider/decider_multiple_temperatures.h>
PLUGINLIB_EXPORT_CLASS(decider::DeciderMultipleTemperatures, decider::Decider)
namespace decider {
    DeciderMultipleTemperatures::DeciderMultipleTemperatures()
        : DeciderBoltzmannCorrection() {
            ros::NodeHandle nh;
            int distance_states, scanner_states;
            do {
            ros::Duration(1).sleep();
            ROS_INFO("[DeciderMultipleTemperatures::DeciderMultipleTemperatures] Waiting for total_states parameter");
            }while (ros::ok() && !nh.hasParam("total_states"));
            nh.param("total_states", distance_states,64);
            ROS_INFO("[DeciderMultipleTemperatures::DeciderMultipleTemperatures] total_states: %d",distance_states);
            nh.param("scanner_states", scanner_states, 1); 
            temperatures_.assign(distance_states*scanner_states, temperature_);
    }
    DeciderMultipleTemperatures::~DeciderMultipleTemperatures() {

    }
    void DeciderMultipleTemperatures::getAction(const rl_msgs::DeciderInput &d_input) {
        int64_t state = d_input.state.content.as_integer[0];
        double temp = temperatures_[state];
        this->getOldAction(d_input.sv_function,temp);
        std::cout << "[DeciderMultipleTemperatures::getAction] v-function for state : "
            << state << ", temp: " << temp << std::endl;
        std::cout << "v: [ ";
        for (auto sv : d_input.sv_function) {
            std::cout << sv << " ";
        }
        std::cout << "]" << std::endl;
        temperatures_[state] = temp*lambda_;        
    }
}