#include <pluginlib/class_list_macros.h>
#include <decider/decider_loop_evasion.h>
PLUGINLIB_EXPORT_CLASS(decider::DeciderLoopEvasion, decider::Decider)

namespace decider {
    DeciderLoopEvasion::DeciderLoopEvasion() :
     DeciderBoltzmannCorrection()
    {
        loop_detector_ = std::unique_ptr<LoopDetector>(new LoopDetector(temperature_, lambda_));
    }

    DeciderLoopEvasion::~DeciderLoopEvasion() { }

    void DeciderLoopEvasion::getAction(const rl_msgs::DeciderInput &d_input) {
        temperature_ = loop_detector_->detectLoop(d_input);
        this->getOldAction(d_input.sv_function,temperature_);
        std::cout << "[DeciderLoopEvasion::getAction] v-function for state : "
            << d_input.state.content.as_integer[0] << ", temp: " << temperature_ << std::endl;
        std::cout << "v: [ ";
        for (auto sv : d_input.sv_function) {
            std::cout << sv << " ";
        }
        std::cout << "]" << std::endl;
    }
}