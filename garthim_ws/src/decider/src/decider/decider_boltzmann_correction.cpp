#include <pluginlib/class_list_macros.h>
#include <decider/decider_boltzmann_correction.h>
PLUGINLIB_EXPORT_CLASS(decider::DeciderBoltzmannCorrection, decider::Decider)
namespace decider {
    DeciderBoltzmannCorrection::DeciderBoltzmannCorrection() : Decider() {
        ros::NodeHandle nh,nh_private("~");
        nh_private.param("lambda", lambda_, 0.9999);
        nh_private.param("temperature", temperature_, 50.0);
        //sub_sv_function_ = nh.subscribe("state_value_function", 100, &DeciderBoltzmannCorrection::getAction, this);
        goal_.action.as_integer.assign(1,0);
    }
    DeciderBoltzmannCorrection::~DeciderBoltzmannCorrection() {}
    void DeciderBoltzmannCorrection::getAction(const rl_msgs::DeciderInput &d_input) {
        getOldAction(d_input.sv_function, this->temperature_);
        std::stringstream ss("[DeciderMultipleTemperatures::getAction] v-function for state : ");
        ss << d_input.state.content.as_integer[0] << ", temp: " << temperature_ << std::endl;
        ss << "v: [ ";
        for (auto sv : d_input.sv_function) {
            ss << sv << " ";
        }
        ss << "]" << std::endl;
        ROS_DEBUG("%s", ss.str().c_str());
        this->temperature_ *= lambda_;
        return;
        const auto sv_f = d_input.sv_function;
        const double max = *std::max_element(sv_f.cbegin(), sv_f.cend());
        const double sum = std::accumulate(sv_f.cbegin(), sv_f.cend(),0.0, [this,&max](double ac,double cur) {
            return ac+exp((cur-max)/temperature_);
        });
        std::vector<double> probabilities(sv_f.size(), 1.0);
        for(size_t i = 0; i < sv_f.size() - 1; i++) {
            probabilities[i] = exp((sv_f[i] - max)/temperature_)/sum;
            probabilities[sv_f.size() - 1] -= probabilities[i];
        }
        double upper_bound = ran_gen_.uniform01(), acc_prob=0;
        uint64_t count = 0;
        bool id_found = false;
        while(count < probabilities.size() && !id_found) {
            if (acc_prob <= upper_bound && upper_bound < acc_prob + probabilities[count]) {
                id_found = true;
            } else {
                acc_prob += probabilities[count++];
            }
        }
        temperature_ *= lambda_;
        goal_.action.as_integer[0] = count;
        ac_.sendGoal(goal_);
    }

    void DeciderBoltzmannCorrection::getOldAction(const std::vector<double> &sv_function, const double temp) {
        //función que calcula la acción a realizar, según distribución de boltzmann
        int a=0; 
        //calculamos probabilidad de cada acción
        double sum =0;
        std::vector<std::vector<double>> Q;
        Q.push_back(sv_function);
        int s = 0;
        std::vector<double> Prob(sv_function.size());
        if(s < Q.size())
        {
            double Qmax = *std::max_element(Q.at(s).begin(), Q.at(s).end());
            for(std::size_t i=0;i<Q.at(s).size();i++)
            {
                sum = sum + exp((Q.at(s).at(i) - Qmax)/temp);
            }
            double accumulative_prob=0;
            for(std::size_t i=0;i<Q.at(s).size()-1;i++)
            {
                if(i < Prob.size()) {
                    Prob.at(i)= exp((Q.at(s).at(i) - Qmax)/temp)/sum;
                    accumulative_prob += Prob.at(i);
                }
            }
            Prob.at(Q[s].size()-1) = 1 - accumulative_prob;
        }

        //seleccionamos con la probabilidad seleccionada con el método de monte carlo
        double u =(getRandom()% 100);

        ROS_DEBUG("\n\nRANDOM: %f \n\n", u);
        u = u/100;
        double acumulativeProb =0;
        for(int i = 0; i < Prob.size();i++)
        {
            if( (acumulativeProb <= u) && (u < (acumulativeProb + Prob.at(i)) ) )
            {
                a=i;
            }
            acumulativeProb = acumulativeProb + Prob.at(i);
        }
        goal_.action.as_integer[0] = (uint64_t) a;
        ROS_DEBUG("[DeciderBoltzmannCorrection::getOldAction] Action selected %d", a);
        ac_.sendGoal(goal_);
    }

    int DeciderBoltzmannCorrection::getRandom() {
        rl_msgs::GetRandom serv;
        while (!ros::service::call("get_random",serv)) {
            ros::Duration(0.1).sleep();
        }
        return serv.response.random_number;
    }
}
