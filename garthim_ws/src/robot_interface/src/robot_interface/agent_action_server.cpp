#include <robot_interface/agent_action_server.h>

namespace robot_interface {
    AgentSimpleActionServer::AgentSimpleActionServer() :
     action_server_(nh_,"perform_action",boost::bind(&AgentSimpleActionServer::performAction, this, _1),false),
     ti_loader_("robot_interface", "robot_interface::TaskInterface")
    {
        init();
        ros::NodeHandle nh_private("~");
        std::string impl_name = "gazebo";
        nh_.param("agent_interface", impl_name);
        reward_client_ = nh_.serviceClient<rl_msgs::GetReward>("get_reward");
        sub_occurrences_server_ = nh_.advertiseService("get_suboccurrences", &AgentSimpleActionServer::getSubOccurrences, this);
        sub_record_server_ = nh_.advertiseService("get_subrecord", &AgentSimpleActionServer::getSubRecord, this);
        pub_transition_result_ = nh_.advertise<rl_msgs::TransitionResult>("transition_result",100, false);
        if (nh_.hasParam("continuous")) {
            result_.transition.state.content.as_floating.assign(1,0);
            result_.transition.action.as_floating.assign(1,0);
            result_.transition.next_state.content.as_floating.assign(1,0);
        } else {
            result_.transition.state.content.as_integer.assign(1,0);
            result_.transition.action.as_integer.assign(1,0);
            result_.transition.next_state.content.as_integer.assign(1,0);
        }
        reset_task_interface_ = nh_.advertiseService("reset_task_interface", &AgentSimpleActionServer::reset, this);
        action_server_.start();
    }
    AgentSimpleActionServer::~AgentSimpleActionServer() {
        
    }

    void AgentSimpleActionServer::performAction(const rl_msgs::AgentActionGoalConstPtr goal) {
        double next_state;
        ROS_DEBUG("Into getState");
        is_action_being_performed = true;
        if (has_first_state_) {
            if (has_episode_ended_) {
                result_.transition.state.content.as_integer[0] = 
                    result_.transition.next_state.content.as_integer[1];
            } else {
                result_.transition.state = result_.transition.next_state;
            }
            ROS_DEBUG("Out getState");
            result_.transition.action = goal->action;
            ROS_DEBUG("INTO SEND COMMAND");
            if (goal->agent_id == 1) {
                //TODO
                ROS_ERROR("ACTION PERFORMED BY MODEL NOT IMPLEMENTED");
                ros::WallDuration(10).sleep();
            } else if (goal->agent_id == 0) {
                result_.transition.time_elapsed = task_interface_->performAction(goal->action);
            } else {
                ROS_ERROR("AGENT_ID %d DOES NOT HAVE AN IMPLEMENTATION", int(goal->agent_id));
                ros::WallDuration(10).sleep();
            }
            ROS_DEBUG("OUT SEND COMMAND");
            ROS_DEBUG("Into getNextState");
            task_interface_->getState(result_.transition.next_state);
            ROS_DEBUG("OUT getNextState");

            ROS_DEBUG("IN RECORD");
            recordAction();

            ROS_DEBUG("OUT RECORD");
        } else {
            task_interface_->getState(result_.transition.next_state);
        }
        result_.transition.delay = NAN;
        rl_msgs::GetReward::Request req;
        rl_msgs::GetReward::Response res;
        req.action = result_.transition.action;
        req.state = result_.transition.state;
        req.next_state = result_.transition.next_state;
        ROS_DEBUG("IN rewardCall");
        reward_client_.call(req,res);
        ROS_DEBUG("OUT rewardCall");
        result_.transition.reward = res.reward;
        has_first_state_ = true;
        
        ROS_DEBUG("[AgentSimpleActionServer] s: %ld, a: %ld, S:%ld, r:%f", result_.transition.state.content.as_integer[0],
            result_.transition.action.as_integer[0], result_.transition.next_state.content.as_integer[0], res.reward);

        has_episode_ended_ = result_.transition.next_state.type > 0;
        has_first_state_ = true;
        pub_transition_result_.publish(result_.transition);
        action_server_.setSucceeded(result_);
        is_action_being_performed = false;
        /*
        double reward;
        save visit state+action
        save 6 values in vector
        use char to save data
        */

    }

    void AgentSimpleActionServer::recordAction() {
        //const std::lock_guard<std::mutex> lock(mutex_record_);
        /*
        unsigned char s,a,S,r,d,t;
        s = (unsigned char) result_.transition.state.content.as_integer[0];
        ROS_INFO("s:%d",s+0);
        a = (unsigned char) result_.transition.action.as_integer[0];
        ROS_DEBUG("a:%d",a+0);
        S = (unsigned char) result_.transition.next_state.content.as_integer[0];
        ROS_DEBUG("S:%d",S+0);
        r = (unsigned char) result_.transition.reward*10;
        d = '\0';
        t = ((unsigned char) result_.transition.time_elapsed*100)/100;*/
        
        int64_t s = result_.transition.state.content.as_integer[0];
        if (s > 0) {
            int64_t a = result_.transition.action.as_integer[0];
            occurrences_[s][a]++;
        }
        //record_.push_back({s,a,S,r,d,t});
    }

    bool AgentSimpleActionServer::getSubOccurrences(rl_msgs::GetSubMatrix::Request& req,
            rl_msgs::GetSubMatrix::Response& res) {
            
        //const std::lock_guard<std::mutex> lock(mutex_record_); 
        int count = 0;
        res.ncolumns = req.columns.size();
        res.nrows = req.rows.size();
        res.data.assign(res.nrows*res.ncolumns,0.0);
        for (auto state : req.rows) {
            for (auto action : req.columns) {
                res.data[count++] = occurrences_[state][action];
            }
        }
        return true;

    }

    bool AgentSimpleActionServer::getSubRecord(rl_msgs::GetSubMatrix::Request& req,
            rl_msgs::GetSubMatrix::Response& res) {
        //const std::lock_guard<std::mutex> lock(mutex_record_); 
        int count = 1;
        res.ncolumns = req.columns.size();
        res.nrows = req.rows.size();
        res.data.assign(res.nrows*res.ncolumns,0.0);
        for (auto row : req.rows) {
            for (auto column : req.columns) {
                res.data[count++] = record_[row][column];
            }
        }
        return true;
    }
    void AgentSimpleActionServer::init() {
        ros::NodeHandle nh, nh_private("~");
        std::string task_interface_instance_name =  "not_implemented";
        //nh_private.param("task_interface", task_interface_instance_name);
        task_interface_instance_name = nh_private.param("task_interface", task_interface_instance_name);
        try {
            this->task_interface_ = ti_loader_.createInstance(task_interface_instance_name);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the \"%s\" task_interface plugin, are you sure it is properly registered and that the containing library is built? Exception: %s",
             task_interface_instance_name.c_str(), ex.what());
            exit(1);
        }
        int total_states, total_actions;
        std::vector<int> action_indexes = {0,1,2,3};
        nh.getParam("actions_enabled",action_indexes);
        total_actions = action_indexes.size();
        std::vector<double> durations;
        nh_private.getParam("times_for_action", durations);
        int durations_state = durations.size() < 1? 1:durations.size();
        nh.param("total_states", total_states, 64);
        occurrences_.assign(total_states, std::vector<long unsigned>(total_actions));
        has_first_state_ = false;
        has_episode_ended_ = false;
        is_action_being_performed = false;
    }
    bool AgentSimpleActionServer::reset(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        //init();
        ros::shutdown();
        return true;
    }
}