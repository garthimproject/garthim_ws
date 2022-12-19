#ifndef __TARGET_GENERATOR_SCAFFOLDER_GARTHIM_H__
#define __TARGET_GENERATOR_SCAFFOLDER_GARTHIM_H__
#include <navigation_interface/target_generator_inside_region.h>
#include <std_msgs/UInt64.h>
namespace navigation_interface {
    /**
     * @brief Sub-class of TargetGeneratorInsideRegion which provides the 
     * functionality of applying scaffolding to new targets rising their 
     * difficulty level, therefore placing them more distant to the agent.
     * 
     */
    class TargetGeneratorScaffolder : public TargetGeneratorInsideRegion {
        private:
            /**
             * @brief Flag indicating whether obstacles will be considered.
             * 
             */
            bool obstacles_;
            /**
             * @brief Buffer to compute succesful episodes.
             * 
             */
            std::vector<bool> episodes_status_;
            /**
             * @brief Max size of episodes_status_.
             * 
             */
            int max_episodes_to_count_;
            /**
             * @brief Current episode index inside episodes_status_.
             * 
             */
            int current_episode_;
            /**
             * @brief Episodes that in fact have been completed at the moment.
             * 
             */
            uint64_t actual_episodes_;
            /**
             * @brief Threshold level at which to rise scaffolding level.
             * 
             */
            double threshold_;
            /**
             * @brief Increment in disctance when scaffolding is reached.
             * 
             */
            double distance_increment_;
            /**
             * @brief Maximum distance at which to put targets.
             * 
             */
            double max_distance_target_;
            /**
             * @brief Publisher to send episode at which scaffolding activates.
             * 
             */
            ros::Publisher publisher_;
        public:
            TargetGeneratorScaffolder();
            ~TargetGeneratorScaffolder();
            /**
             * @brief Get next target based on current agent pose and if buffer reaches threshold rise distance of target.
             * 
             * @param robot_pose Current agent pose.
             * @param target out-parameter. Navigation goal.
             * @param status Flag indicating the result of the episode.
             * @return true if scaffolding condition is reached.
             * @return false otherwise
             */
            virtual bool getNewTarget(const geometry_msgs::Pose& robot_pose,
                geometry_msgs::Pose& target, const uint8_t status = 0) override;
    };
}

#endif