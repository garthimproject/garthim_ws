#include <navigation_interface/translator_to_state.h>

namespace navigation_interface {
    void TranslatorToState::initialize() {
        ros::NodeHandle nh("~"), nh_global;
        if (is_discrete_ = nh.hasParam("discrete")) {
            nh.param("radius_levels", this->radius_levels_, 8.0);
            nh.param("radius_goal_states", this->radius_goal_states_, 0.1875);
            nh.param("radius_unit_states", this->radius_unit_states_, 0.35);
            nh.param("radius_long_distance_states", this->radius_long_distance_states_, 2.0);
            radiuses_.assign((int) this->radius_levels_, 0.0);
            if (radius_levels_ > 0) {
                radiuses_[0] = this->radius_goal_states_;
                if (radius_levels_ > 1) {
                    radiuses_[1] = radiuses_[0] + this->radius_unit_states_;
                    for (int i = 2; i<radiuses_.size();i++) {
                        radiuses_[i] = radiuses_[i-1] + this->radius_long_distance_states_;
                    }
                }
            }
            this->radius_start_last_level_ = radiuses_[radiuses_.size()-1];
            nh.param("orientation_levels",this->orientation_levels_, 8.0);
        } else {
            nh.param("radius", this->radius_start_last_level_,1.5);
        }
        robot_pose_.header.frame_id = "odom";
        robot_pose_.header.stamp = ros::Time(0);
        robot_pose_.pose.position.x =robot_pose_.pose.position.y =robot_pose_.pose.position.z = 0;
        robot_pose_.pose.orientation.x =robot_pose_.pose.orientation.y =robot_pose_.pose.orientation.z =0;
        robot_pose_.pose.orientation.w = 1;
        spatial_states_ = (int64_t) (orientation_levels_ * radius_levels_);
        if (is_used_spencer_ = nh.hasParam("spencer_topic")) {
            nh.param("spencer_orientation_levels", spencer_orientation_levels_, 3.0);
            nh.param("spencer_orientation_vel_levels", spencer_orientation_vel_levels_, 2.0);
            nh.param("spencer_orientation_vel_unit", spencer_orientation_vel_unit_, 0.30);
            nh.param("spencer_depth_levels", spencer_depth_levels_, 3.0);
            nh.param("spencer_depth_unit", spencer_depth_unit_, 0.30);
            nh.param("spencer_depth_collision", spencer_depth_collision_, 0.50);
            nh.param("spencer_total_persons", spencer_total_persons_, 2.0);
            
            scanner_depth_levels_=1;
            scanner_orientation_levels_=0;
        }
        if (is_used_laser_ = nh.hasParam("laser_topic")) {
            nh.param("scanner_orientation_levels",scanner_orientation_levels_, 4.0);
            nh.param("scanner_depth_levels",scanner_depth_levels_, 3.0);
            nh.param("scanner_depth_unit", scanner_depth_unit_, 0.20);
            nh.param("radius_collision", radius_collision_, 0.30);
            nh.param("is_using_kobuki_bumpers", is_using_bumpers_, false);
            //nh_global.setParam("scanner_states", scanner_orientation_levels_*(scanner_depth_levels_-1) + 1);
            pub_point_laser_ = nh_global.advertise<sensor_msgs::PointCloud>("point_laser", 100, false);
            pub_scan_ = nh_global.advertise<sensor_msgs::PointCloud>("pc_scan", 100, false);
            pc_.points.emplace_back();
            pc_.points.emplace_back();
            pc_.points.emplace_back();
            is_tf_laser_initialized_ = false;
        }

        server_unwrap_state_ = nh_global.advertiseService("unwrap_state", &TranslatorToState::unwrapState, this);
        nh.param("debug_messages_on", debug_, false);
        nh_global.param("time_actions_states", time_action_levels_, 1.0);
    }
    
    TranslatorToState::~TranslatorToState(){}
    
    void TranslatorToState::getStateContinuous(const geometry_msgs::Pose& odometry, rl_msgs::State& state){
        ROS_FATAL("[TranslatorToState::getStateContinuous] Not Implemented");
    }

 

    void TranslatorToState::getStateDiscrete(const geometry_msgs::Pose& odometry, const TranslatorActionToTwist& a_t, rl_msgs::State& translation) {
        double inc_y = target_.pose.position.y - odometry.position.y, inc_x = target_.pose.position.x - odometry.position.x;
        double distance = sqrt( pow(inc_x,2) +pow(inc_y,2));
        auto it = std::lower_bound(radiuses_.cbegin(), radiuses_.cend(),distance);
        int64_t id_radius = std::distance(radiuses_.cbegin(), it);
        if (id_radius >= radiuses_.size()) {
            id_radius = radiuses_.size()-1;
        }
        int64_t id_orientation = getOrientationId(inc_x, inc_y, orientation_levels_,odometry);
        int64_t total_states = orientation_levels_;
        int64_t state = id_orientation + total_states*id_radius;
        total_states *= radius_levels_;
        int64_t index_time_action = a_t.getIdCurrentActionDuration();
        state += total_states*index_time_action;
        translation.content.as_integer[0] = state;
        translation.collision = 0;
        ROS_INFO("[TranslatorToState::getStateDiscrete] odom: %f,%f. target: %f, %f. state: %d",
         odometry.position.x, odometry.position.y,
         target_.pose.position.x, target_.pose.position.y,
         int(state));
    }
    void TranslatorToState::getStateDiscrete(const geometry_msgs::Pose& odometry, const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, const TranslatorActionToTwist& a_t, rl_msgs::State& translation) {
        /*double ld = a_t.getLinearDistanceFromVelocityAndCurrentDuration();
        scanner_depth_unit_ = ld;
        radius_unit_states_ = ld;
        double ad = a_t.getAngularDistanceFromVelocityAndCurrentDuration();
        orientation_levels_ = 2*M_PI / ad; */
        getStateDiscrete(odometry, a_t, translation);
        int64_t state = translation.content.as_integer[0];
        int64_t total_states = radius_levels_*orientation_levels_*time_action_levels_;
        std::vector<int64_t> scan_states;
        getStatesScanner(scan, scan_states);
        uint8_t collision = false;
        for (auto s: scan_states) {
            state += total_states*s;
            total_states *= scanner_depth_levels_;
            if (s == 0) {
                ROS_ERROR("COLLISION DETECTED BY USING LASER AS SOURCE");
            }
            collision = collision + (s==0);
        }
        if (is_using_bumpers_) {
            int64_t bumper_state = getStatesBumper(bumper);
            state += total_states*bumper_state;
            collision = collision + (bumper_state > 0);
        }
        translation.content.as_integer[0] = state;
        uint8_t collision_acc = translation.collision + collision;
        bool overflow = collision_acc < translation.collision;
        translation.collision = overflow? UINT8_MAX:collision_acc;
    }
    void TranslatorToState::getStateDiscrete(const geometry_msgs::Pose& odometry, const geometry_msgs::Pose& odometry_from_topic,
        const spencer_tracking_msgs::TrackedPersons& pers_msg, const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, 
        const TranslatorActionToTwist& a_t, rl_msgs::State& translation) {
        //getStateDiscrete(odometry, a_t, translation);
        //get States spencer
        getStateDiscrete(odometry, scan, bumper, a_t,translation);
        int64_t state = translation.content.as_integer[0];
        int64_t total_states = radius_levels_*orientation_levels_*time_action_levels_
            *pow(scanner_depth_levels_, scanner_orientation_levels_);
        std::vector<int64_t> persons_states;
        //getStatesSpencer(odometry_from_topic,pers_msg,persons_states);
        getStatesSpencer(odometry,pers_msg,persons_states);
        uint8_t collision = false;
        const int64_t states_without_vel = spencer_depth_levels_*spencer_orientation_levels_;
        const int64_t states_ori = spencer_orientation_levels_;
        for (auto s:persons_states) {
            state += total_states*s;
            total_states *= spencer_orientation_levels_*spencer_orientation_vel_levels_*spencer_depth_levels_;
            bool collision_by_spencer = s%states_without_vel < spencer_orientation_levels_;
            bool about_to_collide = (s % states_without_vel)/int(spencer_orientation_levels_) == 1;
            if (collision_by_spencer) {
                ROS_ERROR("COLLISION DETECTED BY USING SPENCER AS SOURCE");
            } else if (about_to_collide) {
                ROS_ERROR("COLLISION ABOUT TO HAPPEN DETECTED BY SPENCER AS SOURCE");
            }
            collision = collision + collision_by_spencer*5;
        }
        translation.content.as_integer[0] = state;
        uint8_t collision_acc = translation.collision + collision;
        bool overflow = collision_acc < translation.collision;
        translation.collision = overflow? UINT8_MAX:collision_acc;
        //ROS_WARN("[TranslatorToState::getStateDiscrete] %lu", state);
    }

    void TranslatorToState::getStatesSpencer(const geometry_msgs::Pose& odometry, const spencer_tracking_msgs::TrackedPersons& persons, std::vector<int64_t>& out) {
        //int64_t total_states = 1;
        
        std::stringstream ss;
        ss << "[TranslatorToState::getStatesSpencer] ";
        
        for (auto p:persons.tracks) {
            double th = tf::getYaw(p.pose.pose.orientation);
            double inc_y = p.pose.pose.position.y - odometry.position.y, inc_x = p.pose.pose.position.x - odometry.position.x;
            double distance = sqrt(inc_x*inc_x + inc_y*inc_y);
            int64_t id_distance = distance <= spencer_depth_collision_ ? 0
               : 1 + floor((distance-spencer_depth_collision_)/spencer_depth_unit_);
            id_distance = id_distance>=spencer_depth_levels_? spencer_depth_levels_ - 1:id_distance;
            int64_t id_orientation = getOrientationId(inc_x, inc_y,spencer_orientation_levels_,odometry);
            /*double xv = p.twist.twist.linear.x;
            double yv = p.twist.twist.linear.y;
            double norm_scal = atan2(yv,xv);
            norm_scal -= 2*M_PI*floor(norm_scal/(2*M_PI)); // to [0,360) degrees
            double orientation_unit = 2*M_PI/spencer_orientation_vel_levels_; // discretization
            int64_t id_orientation_vel = trunc(norm_scal/orientation_unit);*/
            int64_t id_orientation_vel = p.twist.twist.linear.x>0?0
                :-p.twist.twist.linear.x/spencer_orientation_vel_unit_;
            id_orientation_vel = id_orientation_vel >= spencer_orientation_vel_levels_?
                 spencer_orientation_vel_levels_ - 1 : id_orientation_vel;
            double theta = atan2(inc_y,inc_x);
            theta = theta - 2*M_PI*floor(theta/(2*M_PI));
            double orientation = theta - tf::getYaw(odometry.orientation);
            orientation = orientation - 2*M_PI*floor(orientation/(2*M_PI));
            
            ss  << " measure-th: "<< orientation <<", th-s: " << id_orientation
                << " measure-d: " << distance << " d-s: "<< id_distance 
                << " rcc: " << p.twist.twist.linear.x <<  ", v-s: " << id_orientation_vel; 
            ss << " all_grouped "<< id_orientation + spencer_orientation_levels_*id_distance +
            spencer_orientation_levels_*spencer_depth_levels_*id_orientation_vel << std::endl;
            //ROS_INFO("[TranslatorToState::getStatesSpencer] x,y = %f, %f", p.pose.pose.position.x, p.pose.pose.position.y);*/
            out.push_back(id_orientation + spencer_orientation_levels_*id_distance + spencer_orientation_levels_*spencer_depth_levels_*id_orientation_vel);
        }
        ROS_INFO("%s", ss.str().c_str());
        
    }

    void TranslatorToState::getStatesScanner(const sensor_msgs::LaserScan& scan, std::vector<int64_t>& out) {
        if (scanner_orientation_levels_ > 0) {
            int sc_orientation_unit = floor(scan.ranges.size() / (this->scanner_orientation_levels_+1));
            bool collision = false;
            int count = 0, i = sc_orientation_unit; 
            auto it_current = scan.ranges.cbegin();
            auto it_e = scan.ranges.cend();
            int inc = scan.ranges.size()/this->scanner_orientation_levels_;
            std::stringstream ss;
            while (it_current + inc <= it_e) {
                auto it = std::min_element(it_current,it_current + inc);
                int i = std::distance(scan.ranges.cbegin(), it);
                double x,y, depth = *it, angle = scan.angle_min + i*scan.angle_increment;
                transformLaserToCartesian(scan.header,depth, angle, x,y);
                // x,y to state
                int64_t state = getDigitStateForScannerRay(sqrt(x*x + y*y));
                // Uncomment these two lines to see each state on console one second at a time.
                //ROS_INFO("%d %d",count, int(state));
                //ros::WallDuration(1).sleep();
                if (debug_) {
                    ss << std::endl << "state-"<<count<<": "<< state << " depth-before: "<<
                    depth << " depth-after: " << sqrt(x*x + y*y) << " angle: " << angle <<
                    " i: " << i << std::endl;
                    pc_.points[count].x = depth*cos(angle);
                    pc_.points[count].y = depth*sin(angle);
                    pc_.points[count].z = 0;
                }
                out.push_back(state);
                it_current += inc;
                count++;
            }
            if (debug_) {
                ROS_INFO("[TranslatorToState::getStatesScanner] %s", ss.str().c_str());
            }
            if (it_current != it_e) {
                auto it = std::min_element(it_current,it_e);
                int i = std::distance(scan.ranges.cbegin(), it);
                double x,y, depth = *it, angle = scan.angle_min + i*scan.angle_increment;
                transformLaserToCartesian(scan.header,depth, angle, x,y);
                int64_t state = getDigitStateForScannerRay(sqrt(x*x + y*y)); 
                out.push_back(state);
                it_current += inc;
                count++;
            }
            if (debug_) {
                pc_.header = scan.header;
                pc_.header.frame_id = "hokuyo_link";
                pub_point_laser_.publish(pc_);

                pc_scan_.header = scan.header;
                pc_scan_.header.frame_id = "hokuyo_link";
                pc_scan_.points = {};
                for (auto it = scan.ranges.begin(); it != scan.ranges.end(); it++) {
                    double i = std::distance(scan.ranges.begin(), it);
                    double d = *it, th = scan.angle_min + i*scan.angle_increment;
                    pc_scan_.points.emplace_back();
                    pc_scan_.points.back().x = d*cos(th);
                    pc_scan_.points.back().y = d*sin(th);
                }
                pub_scan_.publish(pc_scan_);
            }
        }
    }

    void TranslatorToState::transformLaserToCartesian(const std_msgs::Header& header, const float laser_measure, const float angle, double &x, double &y) {
        geometry_msgs::PointStamped laser_measure_as_3D, from_base_link;
        laser_measure_as_3D.header = header;
        laser_measure_as_3D.point.x = laser_measure*cos(angle);
        laser_measure_as_3D.point.y = laser_measure*sin(angle);
         laser_measure_as_3D.point.z = 0;
        if (!is_tf_laser_initialized_) {
            is_tf_laser_initialized_ = true;
            tf2_ros::TransformListener tf_listener(buf_);
            try {
                tf_laser_ = buf_.lookupTransform("base_footprint", "hokuyo_link", ros::Time(0), ros::Duration(1));
            } catch (tf2::TransformException& ex) {
                ROS_WARN("[TranslatorToState::transformLaserToCartesian] Failure %s\n",
                ex.what());
                tf_laser_ = buf_.lookupTransform("base_footprint", "hokuyo_link", ros::Time(0), ros::Duration(10));
            }
        }
        tf2::doTransform(laser_measure_as_3D,from_base_link,tf_laser_);
        x = from_base_link.point.x;
        y = from_base_link.point.y;
    }
    bool TranslatorToState::unwrapState(rl_msgs::UnwrapStateRequest& req, 
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
}
