#include <navigation_interface/target_generator.h>

namespace navigation_interface {
    TargetGenerator::TargetGenerator() {
        ros::NodeHandle nh("~");
        nh.param("radius_target", this->radius_target_, 0.5);
        is_walls_ = false;
        nh.param("debug_target_generator",debug_, false);
    }
    TargetGenerator::~TargetGenerator() {}
    bool TargetGenerator::isInClearSpace(double x, double y, bool debug) {
        for (int i = 0; i < this->cylinders_poses_.size(); i++) {
            if (pow(x - cylinders_poses_[i].position.x,2) 
                + pow(y - cylinders_poses_[i].position.y,2) < pow(0.30 + 0.30,2)) {
                    if (debug_ || debug) {
                        ROS_DEBUG("is too close to cylinder %d. x=%f, y=%f",i, x,y);
                    }
                return false;
            }
        }
        
        if (is_walls_) {
            double cx = 6.5/2.0 , cy = 5.0/2.0;
            double safety =  0.15 + 0.50 + 0.3; // wallwidth, robot_diameter, max_distance to crash
            double inferior_x = this->walls_.position.x - cx + safety;
            double superior_x = this->walls_.position.x + cx - safety;
            double inferior_y = this->walls_.position.y - cy + safety;
            double superior_y = this->walls_.position.y + cy - safety;
            if (x < inferior_x || superior_x < x ||
                inferior_y > y || superior_y < y) {
                    if (debug_ || debug) {
                        ROS_DEBUG("is too close to walls. x=%f, y=%f", x,y);
                    }
                return false;
            }
        }
        return true;
    }
    bool TargetGenerator::isInsidePedestrian (const double x, const double y) const {
        for (const auto& p : this->ped_poses_) {
            double x_2 = x - p.position.x;
            double y_2 = y - p.position.y;
            x_2*=x_2;y_2*=y_2;
            if (x_2 + y_2 < 0.60*0.60) {
                return true;
            }
        }
        return false;
    }
    bool TargetGenerator::getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status /*= 0*/) {
        double x,y,yaw;
        int count = 0;
        do {
            yaw = (getRandom()% 360);
            if (debug_) {
                ROS_DEBUG("\n\nRANDOM CALL: %f \n\n", yaw);
            }
            yaw = yaw *M_PI/180;
            x = this->radius_target_ * cos(yaw);
            y = this->radius_target_ * sin(yaw);
            count++;
        } while (ros::ok() && count < 20 && !isInClearSpace(x+ robot_pose.position.x,y+ robot_pose.position.y));
        target.position.x = x + robot_pose.position.x;
        target.position.y = y + robot_pose.position.y;
        target.position.z = 0;
        yaw = (getRandom()% 360);
        if (debug_) {
            ROS_DEBUG("\n\nRANDOM: %f \n\n", yaw);
        }
        yaw = yaw *M_PI/180;
        target.orientation = tf::createQuaternionMsgFromYaw(yaw);
        ROS_INFO("[TargetGenerator::getNewTarget] Target ( x:%f, y:%f, yaw:%f )",
            target.position.x, target.position.y, yaw);
        return false;
    }

    int TargetGenerator::getRandom() {
        rl_msgs::GetRandom serv;
        while (!ros::service::call("get_random",serv)) {
            ros::Duration(0.1).sleep();
        }
        return serv.response.random_number;
    }
}