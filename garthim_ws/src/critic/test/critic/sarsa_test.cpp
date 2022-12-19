#include <critic/sarsa.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

namespace critic {
    class SarsaTest : public testing::Test {
        public:
            ros::NodeHandle nh_;
            ros::Publisher pub_transition_;
            ros::Subscriber sub_vfunction_;
            rl_msgs::RLVariable sv_fun;
            SarsaTest(){
                pub_transition_ = nh_.advertise<rl_msgs::TransitionResult>("transition_result",1,false);
                sub_vfunction_ = nh_.subscribe("state_value_function",1,&SarsaTest::receiveSVFunction,this);
                waitForSub();
                waitForPub();
                srand(0);
            }
            void receiveSVFunction(const rl_msgs::RLVariable sv_fun) {
                ROS_ERROR("[SarsaTest::receiveSVFunction] Called. sv_fun_status: %d", int(sv_fun.as_floating.empty()));
                this->sv_fun.as_floating.assign(sv_fun.as_floating.cbegin(),sv_fun.as_floating.cend());
            }
            void waitForSub() {
                while (ros::ok() && pub_transition_.getNumSubscribers() < 1) {
                    ros::Duration(1).sleep();
                } 
            }
            void waitForPub() {
                while (ros::ok() && sub_vfunction_.getNumPublishers() < 1) {
                    ros::Duration(1).sleep();
                } 
            }
            virtual void TestBody(){}
    };
    TEST(SarsaTest, shouldASarsaObjectWithDefParamsNoErrorsMustArise) {
        Sarsa sarsa;
        SarsaTest st;
        ASSERT_EQ(st.sub_vfunction_.getNumPublishers(), 1);
        ASSERT_EQ(st.pub_transition_.getNumSubscribers(), 1);
        SUCCEED();
    }
    TEST(SarsaTest, shouldTransitionBeSentSarsaMustSendSVFunctionForNextState) {
        Sarsa sarsa;
        SarsaTest st;
        rl_msgs::TransitionResult t;
        t.state.content.as_integer = {0};
        t.state.type = 0;
        t.next_state.content.as_integer = {1};
        t.next_state.type = 0;
        t.action.as_integer = {0};
        t.reward = 10;
        ASSERT_EQ(st.sub_vfunction_.getNumPublishers(), 1);
        ASSERT_EQ(st.pub_transition_.getNumSubscribers(), 1);
        st.pub_transition_.publish(t);
        int count = 0;
        while(ros::ok() && count < 5) {
            ros::Duration(0.2).sleep();
            ros::spinOnce();
            count++;
        }
        ASSERT_FALSE(st.sv_fun.as_floating.empty());
    }

    TEST(SarsaTest, shouldTransitionBeSentSarsaMustLearnAccordingly) {
        Sarsa sarsa;
        SarsaTest st;
        rl_msgs::TransitionResult t;
        t.state.content.as_integer = {0};
        t.state.type = 0;
        t.next_state.content.as_integer = {1};
        t.next_state.type = 0;
        t.action.as_integer = {0};
        t.reward = 10;
        ASSERT_EQ(st.sub_vfunction_.getNumPublishers(), 1);
        ASSERT_EQ(st.pub_transition_.getNumSubscribers(), 1);
        st.pub_transition_.publish(t);
        int count = 0;
        while(ros::ok() && count < 5) {
            ros::Duration(0.2).sleep();
            ros::spinOnce();
            count++;
        }
        std::vector<std::vector<double>> Q;
        sarsa.getQ(Q);
        
    }
}