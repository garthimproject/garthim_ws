#ifndef __TARGET_GENERATOR_SIMPLE_OBSTACLE_GARTHIM_H__
#define __TARGET_GENERATOR_SIMPLE_OBSTACLE_GARTHIM_H__
#include <ros/ros.h>
#include <navigation_interface/target_generator.h>
#include <geometry_msgs/Pose.h>
#include <rl_msgs/GetRandom.h>
#include <std_srvs/Empty.h>
namespace navigation_interface {
    /**
     * @brief A sub-class of TargetGenerator where robot is tasked to learn a fixed 
     * short path with a cylinder obstacle in the middle.
     * 
     */
    class TargetGeneratorSimpleObstacle : public TargetGenerator {
        private:
            /**
             * @brief Origin pose of agent.
             * 
             */
            geometry_msgs::Pose origin_;
            /**
             * @brief Goal pose for current episode.
             * 
             */
            geometry_msgs::Pose destiny_;
            /**
             * @brief Path to be learnt by agent.
             * 
             */
            std::vector<geometry_msgs::Pose> path_;
            /**
             * @brief Current pose id from path.
             * 
             */
            int current_id_path_;
        public:
            TargetGeneratorSimpleObstacle();
            ~TargetGeneratorSimpleObstacle();
            /**
             * @brief This is the method that after finishing an episode it forwards the next goal from path.
             * it also resets agent pose if episode failed or path is finished.
             * 
             * @param robot_pose. Current agent pose.
             * @param target. out-parameter. Next goal.
             * @param status. Status of current episode.
             * @return true never. (return value is needed in order to conform to super-class)
             * @return false always.
             */
            virtual bool getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status = 0) override;
    };
}
#endif