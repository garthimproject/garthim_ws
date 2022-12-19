#include <gtest/gtest.h>
#include <navigation_interface/translator_to_state.h>

namespace navigation_interface {
    class TranslatorPoseToStateTest : public testing::Test {
        public:
        unsigned seed = 0;
        TranslatorPoseToStateTest() {
        }
        virtual void TestBody(){}
        
    };

    TEST(TranslatorPoseToStateTest, shouldSeedBeZeroTargetMustBeCreatedCorrectly) {
        TranslatorPoseToStateTest tt;
        TranslatorToState t(tt.seed);
        geometry_msgs::Pose target;
        t.reset();
        t.getTarget(target);
        double x = 0.522;
        double y =  -0.146;
        double theta_expected = 0;
        double theta = tf::getYaw(target.orientation);
        theta -= 2*M_PI*floor(theta/2/M_PI);
        EXPECT_EQ(x,round(target.position.x*1000)/1000);
        EXPECT_EQ(y,round(target.position.y*1000)/1000);
        EXPECT_EQ(theta_expected, round(theta*1000)/1000);
    }

    TEST(TranslatorPoseToStateTest, shouldTargetBeInRadiusStateMustBe56) {
        TranslatorPoseToStateTest tt;
        TranslatorToState t(tt.seed);
        geometry_msgs::Pose target;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        target.position.x = 0.5;
        target.position.y = 0;
        target.position.z = 0;
        t.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation.z =odom.orientation.y =odom.orientation.x = 0;
        odom.orientation.w = 1;
        rl_msgs::RLVariable state;
        //t.getStateDiscrete(odom,state);
        ASSERT_EQ(16,state.as_integer[0]);
    }

    TEST(TranslatorPoseToStateTest, shouldTargetBeInThirdSemiQuadrantAndRobotAtOriginStateMustBe3) {
        TranslatorPoseToStateTest tt;
        TranslatorToState t(tt.seed);
        geometry_msgs::Pose target;
        target.position.x = -0.2;
        target.position.y = 0.1;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        t.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(0);
        rl_msgs::RLVariable state;
        //t.translate(odom,state);
        ASSERT_EQ(11,state.as_integer[0]);
    }

    TEST(TranslatorPoseToStateTest, shouldTargetBeInFourthSemiQuadrantAndRobotAtFifthStateMustBe7) {
        TranslatorPoseToStateTest tt;
        TranslatorToState t(tt.seed);
        geometry_msgs::Pose target;
        target.position.x = -0.1;
        target.position.y = -0.2;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        t.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(3*M_PI/2 - 0.01);
        rl_msgs::RLVariable state;
        //t.translate(odom,state);
        ASSERT_EQ(15, state.as_integer[0]);
    }

    TEST(TranslatorPoseToStateTest, shouldTargetBeInFifthSemiQuadrantRobotAtSeventhAndRadiusLevel4StateMustBe7) {
        TranslatorPoseToStateTest tt;
        TranslatorToState t(tt.seed);
        geometry_msgs::Pose target;
        target.position.x = -1.22;
        target.position.y = -1.3;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        t.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(7*M_PI/4);
        rl_msgs::RLVariable state;
        //t.translate(odom,state);
        ASSERT_EQ(62, state.as_integer[0]);
    }
}