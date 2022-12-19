#include <ros/ros.h>
#include <rl_msgs/DeciderInput.h>
#include <std_srvs/Empty.h>
#include <navigation_interface/translator_to_state.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
std::string robot_model_name_ = "turtlebot";
ros::Publisher pub_pose_;
ros::Publisher pub_point_laser_;
ros::Publisher pub_state_pc_;
double publish_rate_cmd_  = 20;
ros::Publisher pub_cmd_vel_;
ros::Subscriber sub_scanner_, sub_odom_;
nav_msgs::Odometry current_odometry_;
sensor_msgs::LaserScan current_laserscan_;
navigation_interface::CollisionStatus bumper_;
double scanner_orientation_levels_ = 3;
double scanner_depth_levels_ = 6;
double scanner_depth_unit_ = .1875;
bool is_tf_laser_initialized_ = false;
tf2_ros::Buffer buf_;
geometry_msgs::TransformStamped tf_laser_;
sensor_msgs::PointCloud pc_;
sensor_msgs::PointCloud pc_to_publish_;
std::vector<bool> is_pc_to_publish_created_ = {false, false, false};
bool is_emplaced_ = false;
sensor_msgs::LaserScan scan_;
geometry_msgs::Twist cmd_vel_moving_;
double radius_collision_ = 0.30;
std::vector<uint64_t> counters = {0,0,0,0,0,0,0,0,0};
bool first_mean = true, are_diffs_set = false;
double mean_norm = 0;
int N=0;
int action_ = -10;
double last_x, last_y;

ros::CallbackQueue cqueue_;
//const double thresh, const int count
inline double getState(bool collision, double r, int count = 0) {
    double state;
    if (collision) {
        state = 0;
    } else if (r >= radius_collision_){
        r -= radius_collision_;
        double d = floor(r/scanner_depth_unit_);
        if (count != 0) {
            ROS_INFO("r:%f, d:%f",r, d);
        }
        if (d >= scanner_depth_levels_ - 1) {
            state = scanner_depth_levels_ - 2;
        } else {
            state = d;
        }
    }
    return state;
}

bool gimme_action(int action,
             geometry_msgs::Twist& translation) {
        translation.linear.x = translation.linear.y = translation.linear.z = 0;
        translation.angular.x = translation.angular.y = translation.angular.z = 0;
        switch (action) {
            case 0:
                translation.linear.x = 0.1875;
                break;
            case 1:
                translation.angular.z = 0.785398;
                break;
            case 2:
                translation.angular.z = -0.785398;
                break;
            
            default:
                ROS_FATAL("[TranslatorActionToTwist::translateDiscrete] Only %d operations supported.",
                   3);
        }
        return true;
}

double performAction(int action) {
        ros::Time start = ros::Time::now();
        gimme_action(action, cmd_vel_moving_);
        double time_action = 1.0;
        do {
            pub_cmd_vel_.publish(cmd_vel_moving_);
            ros::Rate(publish_rate_cmd_).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < time_action);
        return (ros::Time::now() - start).toSec();
}


inline void transformLaserToCartesian(const std_msgs::Header& header, const float laser_measure, const float angle, double &x, double &y) {
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
                ROS_WARN("[DiscreteWorld::createNewTarget] Failure %s\n",
                ex.what());
                tf_laser_ = buf_.lookupTransform("base_footprint", "hokuyo_link", ros::Time(0), ros::Duration(10));
            }
        }
        tf2::doTransform(laser_measure_as_3D,from_base_link,tf_laser_);
        x = from_base_link.point.x;
        y = from_base_link.point.y;
        
        
}
inline bool checkLaserCollision(const double x, const double y, const double thresh, const int count) {
    
    
    //ROS_INFO("x:%f, y;%f, b1:%f, r_2:%f", x,y,pow(x,2) + pow(y,2), pow(0.30,2));
    return pow(x,2) + pow(y,2) < pow(radius_collision_,2);
}
void createPC (int count, double r, double th) {
    if (!is_emplaced_) {
        is_emplaced_ = true;
        pc_to_publish_.channels.emplace_back();
        pc_to_publish_.channels[0].name = "intensity";
    }

    int last_elem = 25;
    if (!is_pc_to_publish_created_[count]) {
        is_pc_to_publish_created_[count] = true;
        for (int i = 0; i <= last_elem; i++) {
            pc_to_publish_.points.emplace_back();
            pc_to_publish_.channels[0].values.emplace_back();
        }
    }
    double inc = r/last_elem;
    double x,y;
    for (int i = 0; i <= last_elem; i++) {
        double r_i = i*inc;
        x = r_i*cos(th);
        y = r_i*sin(th);
        bool collision = checkLaserCollision(x,y,0, count);
        int id = i + count*(last_elem+1);
        pc_to_publish_.points[id].x = x;
        pc_to_publish_.points[id].y = y;
        pc_to_publish_.points[id].z = 0;
        pc_to_publish_.channels[0].values[id] = collision?0:getState(collision, r_i)+1;
    }
    //ROS_INFO("x:%f, y;%f, b1:%f, r_2:%f", x,y,pow(x,2) + pow(y,2), pow(0.30,2));
    
}
int64_t getObstaclesState(const sensor_msgs::LaserScan& scan, const navigation_interface::CollisionStatus& bumper) {
        int sc_orientation_unit = floor(scan.ranges.size() / (scanner_orientation_levels_+1));
        int64_t state = 0, digits = 1;
        bool collision = false;
        int count = 0, i = sc_orientation_unit; 
        while (!collision && i < scan.ranges.size()) {
            double angle = scan.angle_min + i*scan.angle_increment;
            double depth = scan.ranges[i];
            double x,y, r, th;
            transformLaserToCartesian(scan.header, depth,angle,x,y);
            bool current_collision = checkLaserCollision(x,y,0, count);
            collision |= current_collision;
            r = sqrt(x*x + y*y);
            th = atan2(y,x);
            createPC(count, r,th);
            double s = getState(collision, r,1);
            state += int64_t(s*pow(scanner_depth_levels_-1,count));
            i += sc_orientation_unit;
            count++;
        }
        if (!collision) {
            state++;
        }
        pc_to_publish_.header = scan.header;
        pc_to_publish_.header.frame_id = "base_footprint";
        pub_point_laser_.publish(pc_to_publish_);
        return state;
}


void receiveGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id =  "gazebo_model";
    tfs.child_frame_id = "base_footprint";
    for (int i = msg->name.size() - 1; i>=0;i--) {
        if (msg->name.at(i)==robot_model_name_){
            current_odometry_.header.frame_id = "gazebo_model";
            current_odometry_.header.stamp = ros::Time::now();
            current_odometry_.pose.pose = msg -> pose.at(i);
            current_odometry_.twist.twist = msg->twist.at(i);
            pub_pose_.publish(current_odometry_);
            break;
        }
    }
    tfs.transform.translation.x = current_odometry_.pose.pose.position.x;
    tfs.transform.translation.y = current_odometry_.pose.pose.position.y;
    tfs.transform.translation.z = current_odometry_.pose.pose.position.z;
    tfs.transform.rotation.x = current_odometry_.pose.pose.orientation.x;
    tfs.transform.rotation.y = current_odometry_.pose.pose.orientation.y;
    tfs.transform.rotation.z = current_odometry_.pose.pose.orientation.z;
    tfs.transform.rotation.w = current_odometry_.pose.pose.orientation.w;
    tfs.header.stamp = ros::Time::now();
    br.sendTransform(tfs);
}
void receiveLaser(const sensor_msgs::LaserScan& scan) {
    cqueue_.callAvailable();
    if (are_diffs_set) {
        double x = current_odometry_.pose.pose.position.x;
        double y = current_odometry_.pose.pose.position.y;
        double diff_x = x - last_x, diff_y = y - last_y;
        double d = sqrt(diff_x*diff_x + diff_y*diff_y);
        if (action_ == 0) {
            if (first_mean) {
                mean_norm = d;
                first_mean = false;
                N = 1;
            } else {
                N++;
                mean_norm = (mean_norm*(N-1) + d)/N;
            }
        }
        ROS_INFO("MEAAAAAAAAAAN : %f, N: %d", mean_norm,N);
    }
    int64_t state = getObstaclesState(scan_, bumper_);
    scan_ = scan;
    action_ = rand()%3;
    performAction(action_);
    int64_t next_state = getObstaclesState(scan_, bumper_);
    //7
    if (state > 0 && next_state == 0) {
        counters[state]++;
        std::stringstream ss;
        std::string s;
        ss << "[ ";
        for (auto c:counters) {
            ss << c << " ";
        }
        ss << "]";
        s = ss.str();
         ROS_INFO("CHOQUEs %s", s.c_str());
    }
    ROS_INFO("%ld (%d ->) %ld", state,action_, next_state);

    last_x = current_odometry_.pose.pose.position.x;
    last_y = current_odometry_.pose.pose.position.y;
    are_diffs_set = true;
}
void obs(const ros::WallTimerEvent& e) {
    getObstaclesState(scan_, bumper_);
}
int main(int argc, char **argv) {
    ros::init(argc, argv,"scanner_node_test");
    ROS_INFO("scanner_node_test node STARTED");
    
    ros::NodeHandle nh_odom, nh_queue;
    nh_queue.setCallbackQueue(&cqueue_);
    std::string topic = "laserscan";
    std::string namespace_gazebo = nh_odom.param("gazebo_namespace",std::string("gazebo"));
    std::string odom_gazebo_topic = "/"+namespace_gazebo+"/model_states";
    sub_scanner_ = nh_odom.subscribe(topic.c_str(), 1, receiveLaser);
    pub_point_laser_ = nh_odom.advertise<sensor_msgs::PointCloud>("point_laser", 100, false);
    pub_state_pc_ = nh_odom.advertise<sensor_msgs::PointCloud>("point_laser_state", 100, false);
    pub_pose_ = nh_odom.advertise<nav_msgs::Odometry>("my_odom", 100, false);
    pc_.points.emplace_back();
    pc_.points[0].x = pc_.points[0].y = pc_.points[0].z = 0;
    pc_.channels.emplace_back();
    pc_.channels[0].name="intensity";
    pc_.channels[0].values.emplace_back();
    sub_odom_ = nh_queue.subscribe(odom_gazebo_topic.c_str(), 1, receiveGazeboModelStates);
    nh_odom.param("scanner_depth_unit",scanner_depth_unit_, scanner_depth_unit_);
    nh_odom.param("scanner_depth_levels",scanner_depth_levels_, 6.0);
    pub_cmd_vel_ = nh_odom.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100, false);
    //ros::WallTimer timer_ = nh_odom.createWallTimer(ros::WallDuration(1), obs);
    
    /*while (ros::ok()) {
        ros::WallDuration(1).sleep();
        ros::spinOnce();
    }*/

    ros::spin();
    return 0;
}