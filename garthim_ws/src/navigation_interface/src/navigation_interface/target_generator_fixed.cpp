#include <navigation_interface/target_generator_fixed.h>
#include <sensor_msgs/PointCloud.h>

namespace navigation_interface {
    TargetGeneratorFixed::TargetGeneratorFixed() : TargetGenerator() {
        ros::NodeHandle nh("~");
        grid_created_ = false;
        nh.param("height_walls", height_walls_, 6.5);
        nh.param("width_walls", width_walls_, 5.0);
        nh.param("depth_walls", depth_walls_, 0.15);
        if (debug_) {
            pub_grid_ = nh.advertise<sensor_msgs::PointCloud>("grid", 1000, false);
            pub_filtered_ = nh.advertise<sensor_msgs::PointCloud>("filtered", 1000, false);
        }
    }
    TargetGeneratorFixed::~TargetGeneratorFixed() {}
    bool TargetGeneratorFixed::getNewTarget(const geometry_msgs::Pose& robot_pose,
            geometry_msgs::Pose& target, const uint8_t status /*= 0*/) {
        if (!grid_created_) {
            createGrid();
            grid_created_ = true;
        }
        std::vector<geometry_msgs::Pose> filtered;
        std::copy_if(grid_.cbegin(), grid_.cend(), std::back_inserter(filtered),
            [this,robot_pose](const geometry_msgs::Pose& p){
                double x = p.position.x - robot_pose.position.x;
                double y = p.position.y - robot_pose.position.y;
                double x_2y_2 = pow(x,2) + pow(y,2), r_2 = pow(radius_target_, 2);
                return x_2y_2 > 0.30 && x_2y_2 < r_2 + 0.2;
        });
        if (debug_) {
            sensor_msgs::PointCloud pc;
            pc.header.frame_id = "odom";
            pc.header.stamp = ros::Time::now();
            for (auto p: filtered) {
                double x = p.position.x;
                double y = p.position.y;
                pc.points.push_back(geometry_msgs::Point32());
                pc.points.back().x = x;
                pc.points.back().y = y;
                pc.points.back().z = 0;
            }
            pub_filtered_.publish(pc);
        }
        
        if (!filtered.empty()) {
            int id_pose = getRandom()%filtered.size();
            target.position.x = filtered[id_pose].position.x;
            target.position.y = filtered[id_pose].position.y;
        } else {
            int id_pose = getRandom()%grid_.size();
            target.position.x = grid_[id_pose].position.x;
            target.position.y = grid_[id_pose].position.y;
        }
        target.position.z = 0;
        ROS_INFO("[TargetGeneratorFixed::getNewTarget] Target ( x:%f, y:%f )",
            target.position.x, target.position.y);
        return false;
        
    }
    void TargetGeneratorFixed::createGrid() {
        double c_wallsx = this->walls_.position.x, c_wallsy = this->walls_.position.y;
        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "odom";
        pc.header.stamp = ros::Time::now();
        for (double i = 0; i < height_walls_; i+=0.1) {
            double x = c_wallsx - height_walls_/2.0 + i;
            for (double j = 0; j < width_walls_; j+=0.1) {
                double y = c_wallsy - width_walls_/2.0 + j;
                if (isInClearSpace(x,y, false)) {
                    if (debug_) {
                        pc.points.push_back(geometry_msgs::Point32());
                        pc.points.back().x = x;
                        pc.points.back().y = y;
                        pc.points.back().z = 0;
                    }
                    grid_.emplace_back(geometry_msgs::Pose());
                    grid_.back().position.x = x;
                    grid_.back().position.y = y;
                    //ROS_WARN("[TargetGeneratorFixed::createGrid] success. x=%f, y=%f",x,y);
                }
            }
        }
        if (debug_) {
            pub_grid_.publish(pc);
        }
        ROS_INFO("[TargetGeneratorFixed::createGrid] success. size:%lux2",grid_.size());
    }
}