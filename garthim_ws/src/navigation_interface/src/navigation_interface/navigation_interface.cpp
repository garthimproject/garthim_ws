#include <pluginlib/class_list_macros.h>
#include <navigation_interface/navigation_interface.h>
PLUGINLIB_EXPORT_CLASS(navigation_interface::NavigationInterface, robot_interface::TaskInterface)
namespace navigation_interface {
    NavigationInterface::NavigationInterface(){
        ros::NodeHandle nh, nh_priv("~"), nh_odom;
        nh_odom.setCallbackQueue(&queue_);
        if (is_episodic_ = nh_priv.hasParam("episodic")) {
            nh_priv.param("max_steps_per_episode", max_steps_per_episode_, 250);
        }
        robot_model_name_ = nh.param("robot_model_name", robot_model_name_ = "crumb");
        nh_priv.param("publish_rate_cmd", publish_rate_cmd_, 100.0);
        nh_priv.param("after_sending_command", this->after_sending_command_,0);
        if (nh_priv.hasParam("continuous")) {
            state_.content.as_floating.assign(1,0);
        } else {
            state_.content.as_integer.assign(1,0);
        }


        bool has_gazebo = nh.hasParam("gazebo_namespace");
        if (has_gazebo) {
            std::string namespace_gazebo = nh.param("gazebo_namespace",std::string("gazebo"));
            std::string odom_gazebo_topic = "/"+namespace_gazebo+"/model_states";
            // nh.subscribe(odom_gazebo_topic.c_str(), 1, &NavigationInterface::receiveGazeboModelStates, this);
            sub_odom_ = nh_odom.subscribe(odom_gazebo_topic.c_str(), 1, &NavigationInterface::receiveGazeboModelStates, this);
            sub_obstacles_ = nh.subscribe(odom_gazebo_topic.c_str(), 1, &NavigationInterface::receiveGazeboObstacles, this);
            is_spencer_used_simulated_ = nh_priv.hasParam("spencer_topic");
        } else {
            
            sub_odom_ = nh_odom.subscribe("odom", 1,&NavigationInterface::receiveOdometry,this);
            is_spencer_used_simulated_ = false;
        }
        pub_cmd_vel_ = nh_odom.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000, false);
        pub_target_ = nh.advertise<geometry_msgs::PoseStamped>("target",1, false);
        current_odometry_.pose.pose.position.x = NAN;
        current_odometry_.twist.twist.linear.x =current_odometry_.twist.twist.linear.y =current_odometry_.twist.twist.linear.z = NAN;
        current_odometry_.twist.twist.angular.x = current_odometry_.twist.twist.angular.y = current_odometry_.twist.twist.angular.z = NAN;
        pub_current_state_ = nh.advertise<rl_msgs::State>("/state", 100, false);
        pub_pose_ = nh.advertise<nav_msgs::Odometry>("/mario_nostop", 100, false);
        if (is_spencer_used_ = nh_priv.hasParam("spencer_topic")) {
            std::string topic = nh_priv.param("spencer_topic", std::string("spencer_topic"));
            if (!is_spencer_used_simulated_) {
                sub_spencer_ = nh_priv.subscribe(topic.c_str(), 1, &NavigationInterface::receiveSpencerPersons, this);
            }
            nh_priv.param("spencer_total_persons", spencer_total_persons_, 0);
            deriv_dist_ped_.assign(spencer_total_persons_,0.0);
            for (int i = 0; i < spencer_total_persons_; i++) {
                detected_persons_msg_.tracks.emplace_back();
                detected_persons_msg_.tracks[i].pose.pose.position.x = 1000;
                detected_persons_msg_.tracks[i].pose.pose.position.y = 1000;
                detected_persons_msg_.tracks[i].pose.pose.position.z = 0;
                
                detected_persons_msg_.tracks[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                detected_persons_msg_.tracks[i].twist.twist.angular.z = 0;
            }
            if (has_gazebo && !is_spencer_used_simulated_) {
                sub_odom_topic_ = nh_odom.subscribe("odom", 1,&NavigationInterface::receiveOdometry,this);
            }
        }
        if (is_laser_equipped_= nh_priv.hasParam("laser_topic")) {
            std::string topic = nh_priv.param("laser_topic", std::string("laser_scan"));
            sub_scanner_ = nh_odom.subscribe(topic.c_str(), 1, &NavigationInterface::receiveLaser, this);
            sub_bumper_ = nh_odom.subscribe("/mobile_base/events/bumper", 1, &NavigationInterface::receiveBumperEvent, this);
        }
        current_collision_status_.is_received = false;
    }
    NavigationInterface::~NavigationInterface(){}
    inline bool NavigationInterface::reset(const uint8_t status /* =0 */) {
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "odom";
        target.header.stamp = ros::Time::now();
        bool res = translator_to_state_.reset(status);
        translator_to_state_.getTarget(target.pose);
        pub_target_.publish(target);
        steps_in_current_episode_ = 0;
        return res;

    }
    double NavigationInterface::performAction(const rl_msgs::RLVariable& action) {
        ros::Time start = ros::Time::now();
        auto c_time_start = std::chrono::high_resolution_clock::now();
        bool is_movement = translator_from_action_.translate(action,cmd_vel_moving_);
        if (is_movement) {
            double time_action = translator_from_action_.getDurationForAction();
            do {
                pub_cmd_vel_.publish(cmd_vel_moving_);
                ros::Rate(publish_rate_cmd_).sleep();
            } while (ros::ok() && (ros::Time::now() - start).toSec() < time_action);
            if (after_sending_command_ > 0) {
                pub_cmd_vel_.publish(cmd_vel_stopping_);
                if (after_sending_command_ > 1) {
                    do {
                        queue_.callAvailable();
                        ros::Rate(publish_rate_cmd_).sleep();
                    } while(ros::ok() && !translator_from_action_.isActionFinished(current_odometry_.twist.twist));
                }
            }
        }
        if (action.as_integer[0] >= 0) {
            steps_in_current_episode_++;
        }
        auto c_time_stop = std::chrono::high_resolution_clock::now();
        auto c_duration = std::chrono::duration_cast<std::chrono::milliseconds>(c_time_stop - c_time_start);
        ROS_WARN("[NavigationInterface::performAction] std::chrono::ms sending commands: %ld", c_duration.count());
        return (ros::Time::now() - start).toSec();
    }
    void NavigationInterface::receiveBumperEvent(const kobuki_msgs::BumperEvent& event) {
        /**/
        current_collision_status_.is_received = true;
        current_collision_status_.event.state = event.state;
        current_collision_status_.event.bumper = event.bumper;

    }

    void NavigationInterface::receiveGazeboObstacles(const gazebo_msgs::ModelStates::ConstPtr msg) {
        for(int i = msg->name.size() - 1; i>=0;i--) {
            if(msg->name.at(i).find("unit_cylinder_0_mario_") != std::string::npos){
                translator_to_state_.setCylinder(msg -> pose.at(i));
                ROS_INFO("[NavigationInterface::receiveGazeboObstacles] name:%s, x=%f, y=%f",
                    msg->name.at(i).c_str(), msg -> pose.at(i).position.x,msg -> pose.at(i).position.y);
            } else if (msg->name.at(i).find("mario_walls") != std::string::npos) {
                //translator_to_state_.setWalls(msg -> pose.at(i));
                ROS_INFO("[NavigationInterface::receiveGazeboObstacles] name:%s, x=%f, y=%f",
                    msg->name.at(i).c_str(), msg -> pose.at(i).position.x,msg -> pose.at(i).position.y);
            } else if(msg->name.at(i)==robot_model_name_){
                current_odometry_.header.frame_id = "odom";
                current_odometry_.header.stamp = ros::Time::now();
                current_odometry_.pose.pose = msg -> pose.at(i);
                current_odometry_.twist.twist = msg->twist.at(i);
                translator_to_state_.setRobotPose(current_odometry_.pose.pose);
                pub_pose_.publish(current_odometry_);
                ROS_INFO("SETTING INITIAL POSE");
            }
        }
        reset();
        sub_obstacles_.shutdown();
    }
    void NavigationInterface::receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr msg) {
        int current_index_pedestrian = 0;
        std::vector<double> prev_ds;
        double prev_odom_x,prev_odom_y;
        prev_odom_x = current_odometry_.pose.pose.position.x;
        prev_odom_y = current_odometry_.pose.pose.position.y;
        for(int i = msg->name.size() - 1; i>=0;i--) {
            if(msg->name.at(i)==robot_model_name_){
                current_odometry_.header.frame_id = "odom";
                current_odometry_.header.stamp = ros::Time::now();
                current_odometry_.pose.pose = msg -> pose.at(i);
                current_odometry_.twist.twist = msg->twist.at(i);
                pub_pose_.publish(current_odometry_);
                //break;
            }
            if (is_spencer_used_simulated_ && (msg->name.at(i)=="0" || msg->name.at(i)=="1")) {
                //ROS_WARN("HOOOOOOOOO%d", int(current_index_pedestrian));
                double prev_x = detected_persons_msg_.tracks[current_index_pedestrian].pose.pose.position.x
                    - prev_odom_x;
                double prev_y = detected_persons_msg_.tracks[current_index_pedestrian].pose.pose.position.y
                    - prev_odom_y;
                prev_ds.push_back(sqrt(prev_x*prev_x + prev_y*prev_y));
                detected_persons_msg_.tracks[current_index_pedestrian].pose.pose = msg->pose.at(i);
                detected_persons_msg_.tracks[current_index_pedestrian].twist.twist = msg->twist.at(i);
                translator_to_state_.setPedestrian(detected_persons_msg_.tracks[current_index_pedestrian].pose.pose,
                    current_index_pedestrian);
                current_index_pedestrian = (current_index_pedestrian + 1) % spencer_total_persons_;

            }
        }
        if (is_spencer_used_simulated_) {
            int i = 0;
            for (auto p: detected_persons_msg_.tracks) {
                double x_p = p.pose.pose.position.x - current_odometry_.pose.pose.position.x;
                double y_p = p.pose.pose.position.y - current_odometry_.pose.pose.position.y;
                double d = sqrt(x_p*x_p+y_p*y_p);
                //deriv_dist_ped_[i] = d / (d - prev_ds[i]);
                detected_persons_msg_.tracks[i].twist.twist.linear.x = (d - prev_ds[i])/d; // x component of twist angular is our param.
                i++;
            }
        }
    }

    void NavigationInterface::receiveLaser(const sensor_msgs::LaserScan& scan) {
        current_laserscan_ = scan;
    }
    void NavigationInterface::receiveOdometry(const nav_msgs::Odometry& odom) {
        if (!is_spencer_used_) {
            current_odometry_ = odom;
        } else {
            current_odometry_from_topic_ = odom;
        }
    }
    void NavigationInterface::getState(rl_msgs::State& translation) {
        double x = current_odometry_.pose.pose.position.x;
        double y = current_odometry_.pose.pose.position.y;
        double yaw = tf::getYaw(current_odometry_.pose.pose.orientation);
        ROS_DEBUG("yaw %f, yaw-c %f", yaw, yaw - 2*M_PI*floor(yaw/(2*M_PI)));
        yaw -= 2*M_PI*floor(yaw/(2*M_PI));
        queue_.callAvailable();
        x = current_odometry_.pose.pose.position.x - x;
        y = current_odometry_.pose.pose.position.y - y;
        double yaw1 = tf::getYaw(current_odometry_.pose.pose.orientation);
        yaw1 -= 2*M_PI*floor(yaw1/(2*M_PI));
        yaw = yaw1 < yaw? 2*M_PI+yaw1 - yaw: yaw1-yaw;
        N_++;
        double d = x*x+y*y;
        m2_distance_ += d; sum_distance_ += sqrt(d);
        m2_yaw_ += yaw*yaw; sum_yaw_+=yaw;
        double mean_d = sum_distance_/N_, mean_yaw = sum_yaw_/N_;
        ROS_DEBUG("movement performed %f +- %f meters.",mean_d, sqrt(m2_distance_/N_ - mean_d*mean_d));
        ROS_DEBUG("movement performed %f +- %f rads.",mean_yaw, sqrt(m2_yaw_/N_ - mean_yaw*mean_yaw));
        if (is_spencer_used_) {
            translator_to_state_.translate(current_odometry_.pose.pose, current_odometry_from_topic_.pose.pose,
                detected_persons_msg_, current_laserscan_,
                current_collision_status_,translator_from_action_,state_);
        } else if (is_laser_equipped_) {
            translator_to_state_.translate(current_odometry_.pose.pose, current_laserscan_,
                current_collision_status_,translator_from_action_,state_);
        } else {
            translator_to_state_.translate(current_odometry_.pose.pose, translator_from_action_, state_);
        }
        
        pub_current_state_.publish(state_);
        if (state_.content.as_floating.size() > 0) {
            translation.content.as_floating[0] = state_.content.as_floating[0];
        } else {
            translation.content.as_integer[0] = state_.content.as_integer[0];
        }
        translation.collision = state_.collision;
        translation.type = getType(state_.content);
        if (translation.type) {
            bool scaffolding = reset(translation.type);
            if (is_spencer_used_) {
                translator_to_state_.translate(current_odometry_.pose.pose, current_odometry_from_topic_.pose.pose,
                    detected_persons_msg_, current_laserscan_,
                    current_collision_status_,translator_from_action_,state_);
            } else if (is_laser_equipped_) {
                translator_to_state_.translate(current_odometry_.pose.pose, current_laserscan_,
                    current_collision_status_,translator_from_action_,state_);
            } else {
                translator_to_state_.translate(current_odometry_.pose.pose, translator_from_action_, state_);
            }
            if (state_.content.as_floating.size() > 0) {
                if (translation.content.as_floating.size()<2) {
                    translation.content.as_floating.push_back(0);
                }
                translation.content.as_floating[1] = state_.content.as_floating[0];
            } else {
                if (translation.content.as_integer.size()<2) {
                    translation.content.as_integer.push_back(0);
                }
                translation.content.as_integer[1] = state_.content.as_integer[0];
            }
            translation.scaffolding = scaffolding;
            // obtain state
        } else {
            translation.scaffolding = 0;
        }
    }
    void NavigationInterface::receiveSpencerPersons(const spencer_tracking_msgs::TrackedPersons& msg) {
        /*
        std::stringstream ss;
        for (auto it = msg.tracks.begin(); it < msg.tracks.end(); it++) {
            ss << it->detection_id <<": " << it->pose.pose.position.x << ", " << it->pose.pose.position.y << std::endl;
        }
        ROS_DEBUG("Persons received:\n %s", ss.str().c_str());*/
        detected_persons_msg_.header = msg.header;
        for (int i = 0; i < spencer_total_persons_; i++) {
            if (i < msg.tracks.size()) {
                detected_persons_msg_.tracks[i] = msg.tracks[i];
            } else {
                detected_persons_msg_.tracks[i].pose.pose.position.x = 1000;
                detected_persons_msg_.tracks[i].pose.pose.position.y = 1000;
                detected_persons_msg_.tracks[i].twist.twist.angular.z = 0;
            }
        }
        
    }
}