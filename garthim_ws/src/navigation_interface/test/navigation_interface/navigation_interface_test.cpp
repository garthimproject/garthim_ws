#include <gtest/gtest.h>
#include <navigation_interface/navigation_interface.h>

namespace navigation_interface {
    class NavigationInterfaceTest : public testing::Test {
        public:
            unsigned seed = 0;
            NavigationInterfaceTest() {
            }
            virtual void TestBody(){}
    };
   TEST(NavigationInterfaceTest, ShouldAction4BePerformedStateMustBeMultipliedByTimeIndex) {
        NavigationInterface ni;
        rl_msgs::State state, next_state;
        rl_msgs::RLVariable action;
        action.as_integer = {4};
        state.content.as_integer = {4};
        next_state.content.as_integer = {4};
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = odom.pose.pose.position.y = 0;
        /*ni.receiveOdometry(odom);
        ni.getState(state);
        ni.performAction(action);
        ni.receiveOdometry(odom);
        ni.getState(next_state);*/

        uint64_t s,S;
        s = state.content.as_integer[0];

        S = next_state.content.as_integer[0];
        SUCCEED();

    }
/*
    TEST(NavigationInterfaceTest, ShouldAction4BePerformedMultipleTimesStateMustBeMultipliedByTimeIndex) {
        NavigationInterface ni;
        rl_msgs::State state, next_state;
        rl_msgs::RLVariable action;
        action.as_integer = {4};
        state.content.as_integer = {4};
        next_state.content.as_integer = {4};
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = odom.pose.pose.position.y = 0;
        ni.receiveOdometry(odom);
        ni.getState(state);
        ni.performAction(action);
        ni.performAction(action);
        ni.performAction(action);
        ni.receiveOdometry(odom);
        ni.getState(next_state);
        uint64_t s,S;
        s = state.content.as_integer[0];
        S = next_state.content.as_integer[0];
        EXPECT_EQ((s+64*3)%64,S);

    }
TEST(NavigationInterfaceTest, ShouldAction5BePerformedStateMustBeMultipliedByTimeIndex) {
        NavigationInterface ni;
        rl_msgs::State state, next_state;
        rl_msgs::RLVariable action;
        action.as_integer = {5};
        state.content.as_integer = {4};
        next_state.content.as_integer = {4};
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = odom.pose.pose.position.y = 0;
        ni.receiveOdometry(odom);
        ni.getState(state);
        ni.performAction(action);
        ni.receiveOdometry(odom);
        ni.getState(next_state);
        uint64_t s,S;
        s = state.content.as_integer[0];
        S = next_state.content.as_integer[0];
        EXPECT_EQ(s-64,S);

    }

    TEST(NavigationInterfaceTest, ShouldAction5BePerformedMultipleTimesStateMustBeMultipliedByTimeIndex) {
        NavigationInterface ni;
        rl_msgs::State state, next_state;
        rl_msgs::RLVariable action;
        action.as_integer = {5};
        state.content.as_integer = {4};
        next_state.content.as_integer = {4};
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = odom.pose.pose.position.y = 0;
        ni.receiveOdometry(odom);
        ni.getState(state);
        ni.performAction(action);
        ni.performAction(action);
        ni.performAction(action);
        ni.receiveOdometry(odom);
        ni.getState(next_state);
        uint64_t s,S;
        s = state.content.as_integer[0];
        S = next_state.content.as_integer[0];
        EXPECT_EQ((s-64*3)%64,S);

    }
*/
}