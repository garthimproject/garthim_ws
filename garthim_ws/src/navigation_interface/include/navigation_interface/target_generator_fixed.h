#ifndef __TARGET_GENERATOR_FIXED_GARTHIM_H__
#define __TARGET_GENERATOR_FIXED_GARTHIM_H__
#include <ros/ros.h>
#include <navigation_interface/target_generator.h>
#include <geometry_msgs/Pose.h>
#include <rl_msgs/GetRandom.h>
#include <random_numbers/random_numbers.h>
namespace navigation_interface {
    /**
     * @brief A sub-class of target generator that selects goals outside of 
     * a fixed radius around current agent pose.
     * 
     */
    class TargetGeneratorFixed : public TargetGenerator {
        private:
            /**
             * @brief (Debuggin purposes) A publisher to output a point cloud of a grid of points 
             * created inside th walls of the scene. Poses that are deemed too close to obstacles
             * are also filtered out.
             * 
             */
            ros::Publisher pub_grid_;
            /**
             * @brief (Debuggin purposes) A publisher to output a point cloud around the agent pose.
             * This point cloud is obtained from the grid of pint previously computed from the scene and obstacles.
             * 
             */
            ros::Publisher pub_filtered_;
            /**
             * @brief Flag indicating whether first grid of obstacles has already been generated.
             * 
             */
            bool grid_created_;
            /**
             * @brief Grid computed at the beginning of the execution containing all the possible positions where
             * to set navigation goals.
             * 
             */
            std::vector<geometry_msgs::Pose> grid_;
            /**
             * @brief RNG wrapper
             * 
             */
            random_numbers::RandomNumberGenerator ran_gen_;
            /**
             * @brief Paraneter indicating the height of walls.
             * 
             */
            double height_walls_;
            /**
             * @brief Parameter indicating the width of walls
             * 
             */
            double width_walls_;
            /**
             * @brief Parameter indicating the depth of walls;
             * 
             */
            double depth_walls_;
        public:
            TargetGeneratorFixed();
            ~TargetGeneratorFixed();
            /**
             * @brief Get a new target based on the computed grid and the current location of the robot.
             * A radius around the agent is computed and all poses from a fixed distance are choosable.
             * 
             * @param robot_pose Current robot pose.
             * @param target out-parameter. Next goal computed.
             * @param status Status of the current episode.
             * @return true never. (return value is needed in order to conform to super-class)
             * @return false always.
             */
            virtual bool getNewTarget(const geometry_msgs::Pose &robot_pose,
                                      geometry_msgs::Pose &target, const uint8_t status = 0) override;
            /**
             * @brief Method that populates the grid of selectable goal poses at the beginning of the learning process.
             * 
             */
            void createGrid();
    };
}

#endif