#include <model_interface/world_discrete.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cstdio>
namespace robot_interface {
    class ModelInterfaceTest : public testing::Test {
        public:
        ModelInterfaceTest(){
          srand(0);
        }
        virtual void TestBody(){}
        
    };
    TEST(ModelInterfaceTest, shouldSeedBeZeroTargetMustBeCreatedCorrectly) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        geometry_msgs::Pose target;
        double state_returned = dw.createNewTarget(target);
        double robot_x = 1;
        double x = 0.478 + robot_x;
        double y =  -0.146;
        double theta_expected = 0;
        double theta = tf::getYaw(target.orientation);
        theta -= 2*M_PI*floor(theta/2/M_PI);
        EXPECT_DOUBLE_EQ(60, state_returned);
        EXPECT_DOUBLE_EQ(x,round(target.position.x*1000)/1000);
        EXPECT_DOUBLE_EQ(y,round(target.position.y*1000)/1000);
        EXPECT_DOUBLE_EQ(theta_expected, round(theta*1000)/1000);
    }

    TEST(ModelInterfaceTest, shouldTargetBeInRadiusStateMustBe56) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        ros::NodeHandle nh;
        geometry_msgs::Pose target;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        target.position.x = 0.5;
        target.position.y = 0;
        target.position.z = 0;
        dw.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation.z =odom.orientation.y =odom.orientation.x = 0;
        odom.orientation.w = 1;
        double state = dw.getState(odom);
        ASSERT_EQ(state,56);
    }

    TEST(ModelInterfaceTest, shouldTargetBeInThirdSemiQuadrantAndRobotAtOriginStateMustBe3) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        geometry_msgs::Pose target;
        target.position.x = -0.2;
        target.position.y = 0.1;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        dw.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(0);
        double state = dw.getState(odom);
        ASSERT_EQ(state,27);
    }

    TEST(ModelInterfaceTest, shouldTargetBeInFourthSemiQuadrantAndRobotAtFifthStateMustBe7) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        geometry_msgs::Pose target;
        target.position.x = -0.1;
        target.position.y = -0.2;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        dw.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(3*M_PI/2 - 0.01);
        double state = dw.getState(odom);
        ASSERT_EQ(state,31);
    }

    TEST(ModelInterfaceTest, shouldTargetBeInFifthSemiQuadrantRobotAtSeventhAndRadiusLevel4StateMustBe7) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        geometry_msgs::Pose target;
        target.position.x = -1.22;
        target.position.y = -1.3;
        target.position.z = 0;
        target.orientation.x =target.orientation.y =target.orientation.z = 0;
        target.orientation.w = 1;
        dw.setTarget(target);
        geometry_msgs::Pose odom;
        odom.orientation = tf::createQuaternionMsgFromYaw(7*M_PI/4);
        double state = dw.getState(odom);
        ASSERT_EQ(state,62);
    }

    TEST(ModelInterfaceTest,shouldNumbersFrom0to4BeCalledCommandsMustBeCreatedCorrectly) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        for (int i = 0; i < 4; i++) {
            geometry_msgs::Twist cmd_vel;
            dw.getCommand(i, cmd_vel);
            double vlx,vly,vlz,vax,vay,vaz;
            switch(i) {
                case 0:
                    vlx=0.3;
                    vly=vlz=vax=vay=vaz=0;
                    break;
                case 1:
                    vlx=-0.3;
                    vly=vlz=vax=vay=vaz=0;
                    break;
                case 2:
                    vaz=0.9;
                    vly=vlz=vax=vay=vlx=0;
                    break;
                default:
                    vaz=-0.9;
                    vly=vlz=vax=vay=vlx=0;
                    break;
            }

                ASSERT_EQ(cmd_vel.linear.x, vlx);
                ASSERT_EQ(cmd_vel.linear.y, vly);
                ASSERT_EQ(cmd_vel.linear.z, vlz);
                ASSERT_EQ(cmd_vel.angular.x, vax);
                ASSERT_EQ(cmd_vel.angular.y, vay);
                ASSERT_EQ(cmd_vel.angular.z, vaz);
        }
    
    }
    TEST(ModelInterfaceTest,shouldActionNegativeBeIssuedExceptionMustBeThrown) {
        ModelInterfaceTest t;
        DiscreteWorld dw;
        geometry_msgs::Twist cmd_vel;
        EXPECT_ANY_THROW(dw.getCommand(-10, cmd_vel));
    }
}