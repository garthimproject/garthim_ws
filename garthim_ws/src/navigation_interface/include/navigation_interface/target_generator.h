#ifndef __TARGET_GENERATOR_GARTHIM_H__
#define __TARGET_GENERATOR_GARTHIM_H__
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <rl_msgs/GetRandom.h>
#include <tf/tf.h>
namespace navigation_interface {
    /**
     * @brief Class that generates goals for navigation to be performed. Goals are needed to translate real world measures to 
     * reinforcement learning states.
     * 
     */
    class TargetGenerator {
        private:
            /**
             * @brief Whether there are walls present in the scene.
             * 
             */
            bool is_walls_;
            /**
             * @brief Obstacles from the scene abstracted in cylinder form.
             * 
             */
            std::vector<geometry_msgs::Pose> cylinders_poses_;
        protected:

            std::vector<geometry_msgs::Pose> ped_poses_;
            /**
             * @brief Flag indicating if debug messages will be shown.
             * 
             */
            bool debug_;
            /**
             * @brief Distance at which goals are generated.
             * 
             */
            double radius_target_;
            /**
             * @brief Method for calling a rng provider service.
             * 
             * @return int random number.
             */
            static int getRandom();
            /**
             * @brief This method returns whether point (x,y) is not inside obstacles radius.
             * 
             * @param x component x of position (x,y) to check collision.
             * @param y component y of position (x,y) to check collision.
             * @param debug flag indicating if debug messages will be shown.
             * @return true if (x,y) is outside obstacles range.
             * @return false otherwise.
             */
            bool isInClearSpace(double x, double y, bool debug=true);
            bool isInsidePedestrian (const double x, const double y) const;
            geometry_msgs::Pose walls_;
        public:
            TargetGenerator();
            virtual ~TargetGenerator();
            /**
             * @brief Get a target pose for a given agent pose taking into account the status at which 
             * the current episode ended.
             * 
             * @param robot_pose Current pose of the agent.
             * @param target (out param) Target of navigation goal.
             * @param status Status at which the current episode ended.
             * @return true if scaffolding has been performed.
             * @return false otherwise.
             */
            virtual bool getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status = 0);
            /**
             * @brief Set a pose for an obstacle (abstracted as a cylinder) in a scene.
             * 
             * @param pose Pose of the center of an obstacle.
             */
            inline void setCylinder(const geometry_msgs::Pose& pose) {
                cylinders_poses_.emplace_back(geometry_msgs::Pose());
                cylinders_poses_.back().position.x = pose.position.x;
                cylinders_poses_.back().position.y = pose.position.y;
            }
            inline void setPedestrian(const geometry_msgs::Pose& pose, int id) {
                if (id >= ped_poses_.size()) {
                    ped_poses_.emplace_back();
                }
                ped_poses_[id].position.x = pose.position.x;
                ped_poses_[id].position.y = pose.position.y;
            }
            /**
             * @brief Set the center of the walls of a room where the agent operates.
             * 
             * @param pose Center of the walls.
             */
            inline void setWalls(const geometry_msgs::Pose& pose) {
                is_walls_ = true;
                walls_.position.x = pose.position.x;
                walls_.position.y = pose.position.y;
            }
    };
}
#endif