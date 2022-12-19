#include <ros/ros.h>
#include <decider/decider.h>
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>
namespace decider_node {
    class DeciderLoader {
        private:
            pluginlib::ClassLoader<decider::Decider> d_loader_;
            boost::shared_ptr<decider::Decider> decider_, decider_exploitation_;
            std::string decider_instance_name_;
            ros::ServiceServer reset_service_;
            ros::Subscriber sub_sv_function_;
        public:
            DeciderLoader() :
                d_loader_("decider", "decider::Decider") {
                ros::NodeHandle nh_global,nh("~");
                decider_instance_name_ =  "not implemented";
                decider_instance_name_ = nh.param("decider_implementation", decider_instance_name_);
                interfaceLoader(decider_, decider_instance_name_);
                //interfaceLoader(decider_exploitation_, "decider::DeciderMaxPolicy");
                reset_service_ = nh_global.advertiseService("reset_decider", &DeciderLoader::reset, this);
                sub_sv_function_ = nh_global.subscribe("state_value_function", 100, &DeciderLoader::getAction, this);
            }
            bool reset(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
                //interfaceLoader();
                ros::shutdown();
                return true;
            }
            void getAction(const rl_msgs::DeciderInput &d_input) {
                if (true || d_input.is_first_episode_step==1) {
                    decider_->getAction(d_input);
                } else {
                    decider_exploitation_->getAction(d_input);
                }

            }
            inline void interfaceLoader(boost::shared_ptr<decider::Decider>& dec_ptr, const std::string& name) {
                try {
                    dec_ptr = d_loader_.createInstance(name);
                } catch (const pluginlib::PluginlibException& ex) {
                    ROS_FATAL("Failed to create the \"%s\" decider plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
                        decider_instance_name_.c_str(), ex.what());
                        exit(1);
                }
            }
    };
}

int main(int argc, char **argv) {
    ros::init(argc, argv,"decider_node");
    ROS_INFO("decider_node STARTED");
    decider_node::DeciderLoader dl;
    //ros::ServiceServer server = nh.advertiseService("get_action",&getAction);

    ros::spin();
    ROS_INFO("decider_node ENDING...");
    return 0;
}
