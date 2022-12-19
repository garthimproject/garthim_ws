#include <navigation_interface/target_generator_scaffolder.h>

namespace navigation_interface {
    TargetGeneratorScaffolder::TargetGeneratorScaffolder() :
        TargetGeneratorInsideRegion()
     {
        ros::NodeHandle nh("~"), nh_global;
        nh.param("buffer_episode_size_scaffolding", max_episodes_to_count_, 100);
        nh.param("threshold_scaffolding", threshold_, 0.75);
        nh.param("increment_scaffolding", distance_increment_, 0.15);
        nh.param("max_distance_target", max_distance_target_, 6.0);
        nh.param("scaffolder_obstacles", obstacles_, false);
        episodes_status_.assign(max_episodes_to_count_,false);
        current_episode_ = 0;
        actual_episodes_ = 0;
        publisher_ = nh_global.advertise<std_msgs::UInt64>("scaffolding", 100, false);
    }

    TargetGeneratorScaffolder::~TargetGeneratorScaffolder(){}

    bool TargetGeneratorScaffolder::getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status /*= 0 */) {
        bool res = false;
        episodes_status_[current_episode_] = status == 2;
        double sum=0;
        std::for_each(episodes_status_.cbegin(), episodes_status_.cend(), [&sum](bool s) {
            sum += s;
        });
        if (sum/max_episodes_to_count_ >= threshold_ &&
             radius_target_ + distance_increment_ < max_distance_target_) {
            radius_target_ += distance_increment_;
            ROS_INFO("[TargetGeneratorScaffolder::getNewTarget] New Radius distance: %f", radius_target_);
            episodes_status_.assign(max_episodes_to_count_,false);
            std_msgs::UInt64 msg;
            msg.data = actual_episodes_;
            publisher_.publish(msg);
            current_episode_ = 0;
            res = true;
        } else {
            current_episode_ = (current_episode_+1)%max_episodes_to_count_;
        }
        actual_episodes_++;
        if (obstacles_) {
            TargetGeneratorInsideRegion::getNewTarget(robot_pose, target,status);
        } else {
            TargetGenerator::getNewTarget(robot_pose, target,status);
        }
        return res;
    }
}