#include <navigation_interface/target_generator_inside_region.h>
namespace navigation_interface {
    TargetGeneratorInsideRegion::TargetGeneratorInsideRegion()
        :TargetGenerator() {
        ros::NodeHandle nh("~");
        grid_created_ = false;
        regions_x_ = {};
        regions_y_ = {};
        nh.getParam("regions_x",regions_x_);
        nh.getParam("regions_y",regions_y_);
        nh.param("depth_walls", depth_walls_, 0.2);
        nh.param("grid_point_distance", grid_distance_inbetween_, 0.05);
        double x,y,yaw;
        nh.param("robot_pose_x",x,0.0);
        nh.param("robot_pose_y",y,0.0);
        nh.param("robot_pose_yaw",yaw,0.0);
        tf_from_zero_to_robot_initial_.setOrigin(tf::Vector3(x,y,0));
        tf_from_zero_to_robot_initial_.setRotation(tf::createQuaternionFromYaw(yaw));
        tf_from_zero_to_robot_initial_ = tf_from_zero_to_robot_initial_.inverse();
        if (debug_) {
            pub_grid_ = nh.advertise<sensor_msgs::PointCloud>("grid", 1000, false);
            pub_filtered_ = nh.advertise<sensor_msgs::PointCloud>("filtered", 1000, false);
        }
        // If parameter is not set, we assume initial radius is safe for robot
        // i.e. safety_radius_ > robot_radius+0.1 && f.a. th in [0,2pi]. x,y=rcosth,rsinth inside scene.
        nh.param("target_generator_safety_radius", safety_radius_, radius_target_);
    }
    TargetGeneratorInsideRegion::~TargetGeneratorInsideRegion() { }

    bool TargetGeneratorInsideRegion::getNewTarget(const geometry_msgs::Pose& robot_pose,
            geometry_msgs::Pose& target, const uint8_t status /*= 0*/) {
        if (!grid_created_) {
            createGrid();
            grid_created_ = true;
        }
        std::vector<geometry_msgs::Pose> filtered;
        std::copy_if(grid_.cbegin(), grid_.cend(), std::back_inserter(filtered),
            [this,robot_pose](const geometry_msgs::Pose& p){
                bool is_in_ped = isInsidePedestrian(p.position.x,p.position.y);
                double x = p.position.x - robot_pose.position.x;
                double y = p.position.y - robot_pose.position.y;
                double x_2y_2 = pow(x,2) + pow(y,2), r_2 = pow(radius_target_, 2);
                //return x_2y_2 > 0.30 && x_2y_2 < r_2 + 0.2;
                return !is_in_ped && x_2y_2 > r_2 - 0.015 && x_2y_2 < r_2 + 0.015;
        });
        if (filtered.empty()) {
            ROS_WARN("[TargetGeneratorInsideRegion::getNewTarget] filtered EMPTY");
            std::copy_if(grid_.cbegin(), grid_.cend(), std::back_inserter(filtered),
            [this,robot_pose](const geometry_msgs::Pose& p){
                bool is_in_ped = isInsidePedestrian(p.position.x,p.position.y);
                double x = p.position.x - robot_pose.position.x;
                double y = p.position.y - robot_pose.position.y;
                double x_2y_2 = pow(x,2) + pow(y,2), r_2 = pow(safety_radius_, 2);
                //return x_2y_2 > 0.30 && x_2y_2 < r_2 + 0.2;
                return !is_in_ped && x_2y_2 >= r_2;
            });
        }
        if (debug_) {
            sensor_msgs::PointCloud pc;
            pc.header.frame_id = "odom";
            pc.header.stamp = ros::Time::now();
            tf::Vector3 v;
            for (auto p: filtered) {
                double x = p.position.x;
                double y = p.position.y;
                v.setValue(x,y,0);
                v = tf_from_zero_to_robot_initial_*v;
                pc.points.emplace_back();
                pc.points.back().x = v.getX();
                pc.points.back().y = v.getY();
                pc.points.back().z = 0;
            }
            pub_filtered_.publish(pc);    
        }
        int id_pose = getRandom()%filtered.size();
        // 1.089980, 0.257640
        target.position.x = filtered[id_pose].position.x;
        target.position.y = filtered[id_pose].position.y;
        /* target.position.x = 1.089980;
        target.position.y =  0.257640; */
        target.position.z = 0;
        ROS_INFO("[TargetGeneratorInsideRegion::getNewTarget] Target ( x:%f, y:%f )",
            target.position.x, target.position.y);
        return false;
        
    }

    void TargetGeneratorInsideRegion::createGrid() {
        //ros::shutdown();
        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "odom";
        pc.header.stamp = ros::Time::now();
        tf::Vector3 v;
        for (int i = 0; i<int(regions_x_.size())-1; i+=2) {
            double inf_x = regions_x_[i],sup_x = regions_x_[i+1];
            double inf_y = regions_y_[i],sup_y = regions_y_[i+1];
            for (double x = inf_x; x < sup_x; x+=grid_distance_inbetween_) {
                for (double y = inf_y; y < sup_y; y+=grid_distance_inbetween_) {
                    if (isInClearSpace(x,y, false)) {
                        double x_tf, y_tf;
                        v.setValue(x,y,0);
                        v = tf_from_zero_to_robot_initial_*v;
                        if (debug_) {
                            pc.points.emplace_back();
                            pc.points.back().x = v.getX();
                            pc.points.back().y = v.getY();
                            pc.points.back().z = 0;
                        }
                        grid_.emplace_back();
                        grid_.back().position.x = x;
                        grid_.back().position.y = y;
                        //ROS_WARN("[TargetGeneratorInsideRegion::createGrid] success. x=%f, y=%f",x,y);
                    }
                }
            }
        }
        if (debug_) {
            pub_grid_.publish(pc);
        }
        ROS_INFO("[TargetGeneratorInsideRegion::createGrid] success. size:%lux2",grid_.size());
    }
}