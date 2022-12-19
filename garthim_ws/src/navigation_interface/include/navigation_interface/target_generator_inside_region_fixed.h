#ifndef __TARGET_GENERATOR_INSIDE_REGION_FIXED_GARTHIM_H__
#define __TARGET_GENERATOR_INSIDE_REGION_FIXED_GARTHIM_H__
#include <ros/ros.h>
#include <navigation_interface/target_generator.h>
#include <geometry_msgs/Pose.h>
#include <rl_msgs/GetRandom.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/PointCloud.h>
namespace navigation_interface {
    /**
     * @brief A sub-class of TargetGenerator that creates goals inside rectangular areas.
     * 
     */
    class TargetGeneratorInsideRegionFixed : public TargetGenerator {
        private:
            /**
             * @brief A vector storing the x components from the rectangular regions where to compute goals.
             * 
             */
            std::vector<double> regions_x_;
            /**
             * @brief A vector storing the y components from the rectangular regions where to compute goals.
             * 
             */
            std::vector<double> regions_y_;
            /**
             * @brief (Debugging purposes) Publisher of point cloud messages composed of point grid inside regions outside obstacles.
             * 
             */
            ros::Publisher pub_grid_;
            /**
             * @brief (Debugging purposes) Publisher of point cloud messages composed of point grid inside regions outside obstacles around agent radius.
             * 
             */
            ros::Publisher pub_filtered_;
            /**
             * @brief Flag indicating if grid points are already created.
             * 
             */
            bool grid_created_;
            /**
             * @brief Vector containing all poses forming the grid of selectable points by the target generator.
             * 
             */
            std::vector<geometry_msgs::Pose> grid_;
            /**
             * @brief RNG wrapper.
             * 
             */
            random_numbers::RandomNumberGenerator ran_gen_;
            /**
             * @brief Depth of the walls.
             * 
             */
            double depth_walls_;
            /**
             * @brief Factor of granularity between points in the grid.
             * 
             */
            double grid_distance_inbetween_;
            /**
             * @brief TF to transpose initial position to real-world (0,0)
             * 
             */
            tf::Transform tf_from_zero_to_robot_initial_;
            /**
             * @brief A distance measure used to unsure an already reached goal will never be sent.
             * 
             */
            double safety_radius_;
            std::vector<geometry_msgs::Pose> goals_;
            int current_goal_;
        public:
            TargetGeneratorInsideRegionFixed();
            ~TargetGeneratorInsideRegionFixed();
            /**
             * @brief A method to obtain next target inside the pre-defined obstacle region, 
             * purging points deemed too close to obstacles.
             * 
             * @param robot_pose Agent pose.
             * @param target (out-parameter) Target selected.
             * @param status status of last finished episode.
             * @return true never.
             * @return false always.
             */
            virtual bool getNewTarget(const geometry_msgs::Pose &robot_pose,
                                      geometry_msgs::Pose &target, const uint8_t status = 0) override;
            /**
             * @brief Method called at the beginning of the execution to create all points inside the region.
             * 
             */
            void createGrid();
    };
}
#endif