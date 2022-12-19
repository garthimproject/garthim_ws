#include <navigation_interface/target_generator_inside_region_fixed.h>
namespace navigation_interface {
    TargetGeneratorInsideRegionFixed::TargetGeneratorInsideRegionFixed()
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
        current_goal_=0;
        goals_ = {};
        goals_.emplace_back();
        goals_.back().position.x = 11.5;
        goals_.back().position.y = 4.5;
        goals_.emplace_back();
        goals_.back().position.x = 8.0;
        goals_.back().position.y = 1.5;
    }
    TargetGeneratorInsideRegionFixed::~TargetGeneratorInsideRegionFixed() { }

    bool TargetGeneratorInsideRegionFixed::getNewTarget(const geometry_msgs::Pose& robot_pose,
            geometry_msgs::Pose& target, const uint8_t status /*= 0*/) {
        if (!grid_created_) {
            createGrid();
            grid_created_ = true;
        }
        double x = goals_[current_goal_].position.x;
        double y = goals_[current_goal_].position.y;
        double d = sqrt(x*x+y*y);
        const double RADIUS = 0.5;
        double th = double(rand())*2*M_PI/RAND_MAX;
        double r = sqrt(RADIUS*double(rand())/RAND_MAX);
        double x_r = x + r*cos(th);
        double y_r = y + r*sin(th);
        target.position.x = x_r;
        target.position.y = y_r;
        target.position.z = 0;
        ROS_INFO("[TargetGeneratorInsideRegionFixed::getNewTarget] Target ( x:%f, y:%f )",
            target.position.x, target.position.y);
        current_goal_ = (current_goal_+1)%goals_.size();
        return false;
        
    }

    void TargetGeneratorInsideRegionFixed::createGrid() {
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
        ROS_INFO("[TargetGeneratorInsideRegionFixed::createGrid] success. size:%lux2",grid_.size());
    }
}