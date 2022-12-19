#include <ros/ros.h>
#include <handles/PhysicalHandle.h>
#include <handles/MasterHandle.h>
#include <rl_pkg/io/FileHandler.h>
#include <handles/MaxMovement.h>
#include <handles/RewardHandler.h>
#include <sys/stat.h>
#include <fstream>
namespace rl_metrics {
    /**
     * @brief 
     * 
     */
    class MeasureQ {
        private:
            int total_experiments;
            std::string id_experiment_;
            std::unique_ptr<PhysicalHandle> physical_handle;
            std::unique_ptr<MasterHandle> master_handle;
            std::vector<std::vector<double>> end_rewards;
            bool write_each_iteration;
        public:
            MeasureQ();
            /*{
                //PhysicalHandle(double distMax,
                // int distanceLevels, int orientationLevels, 
                //int timeLevels, double actionTime, int numAction,
                // int stateSize, std::vector<double> timeOptions);      

            }*/
            ~MeasureQ(){}
            /**
             * @brief 
             * 
             */
            void writeFileEndRewards();
            /**
             * @brief 
             * 
             */
            void runExperiments();

            const PhysicalHandle &getPhysicalHandle();

            int getTotalExperiments();

    };
}