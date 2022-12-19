#include <ros/ros.h>
#include <gtest/gtest.h>
#include <rl_metrics/rl_metrics.h>
namespace rl_metrics {
    class MeasureQTest : public testing::Test {
        public:
        MeasureQTest(){}
        virtual void TestBody(){}
        // (double distMax, int distanceLevels, int orientationLevels, int timeLevels, double actionTime, int numAction, int stateSize, std::vector<double> timeOptions);      
    
        void measureEmptyConstructor() {
            MeasureQ measure;
            double DISTANCE_MAX = 1.5;      //Distancia maxima al target dist_max
            int DISTANCE_LEVELS = 8;        //Niveles de disatncia al target (estado 0)
            int ORIENTATION_LEVELS = 8;     //Niveles de diferencia de orientación con el target (estado 1)
               // si está seleccionado el aprendizaje con elección de tiempos, este será el tiempo inicial
            double ACTION_TIME =1.0;//0.1;  
            int STATE_SIZE = 2;
            unsigned total = 100;/*
            auto ph = measure.getPhysicalHandle();
            EXPECT_EQ(ph.getDistanceMax(), DISTANCE_MAX);
            EXPECT_EQ(ph.getOrientationLevels(), ORIENTATION_LEVELS);
            EXPECT_EQ(ph.getActionTime(), ACTION_TIME);
            EXPECT_EQ(ph.getStateSize(), STATE_SIZE);
            EXPECT_EQ(measure.getTotalExperiments(), total);*/
            

        }

    };
    TEST(PhysicalHandleTest, shouldEmptyConstructorBeCalledResultsMustBeSetToDefault) {
       MeasureQTest mq;
       mq.measureEmptyConstructor();
    }
}