#include <robot_interface/gazebo_interface.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/tf.h>
#include <gtest/gtest.h>
#include <cstdio>
#include <thread>
namespace robot_interface {
    class GazeboInterfaceTest : public testing::Test, gazebo_msgs::GetModelState {
        public:
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Subscriber sub_t;
        ros::ServiceServer server;
        std::unique_ptr<GazeboInterface> gi_ptr;
        std::unique_ptr<ros::AsyncSpinner> aspinner;
        bool started_receiving=false, ended_receiving=false;
        bool transition_received = false;
        double lm,am, action = NAN, action_result;
        bool getter_service_called = false;
        rl_msgs::TransitionReward transition;
        std::thread t;
        GazeboInterfaceTest() {
            srand(0);
            ros::NodeHandle nh;
            nh.param("linear_velocity_module", lm, 0.3);
            nh.param("angular_velocity_module", am, 0.8);
            sub_t = nh.subscribe("/transition",10,&GazeboInterfaceTest::receiveTransition, this);
            sub = nh.subscribe("/mobile_base/commands/velocity",100,&GazeboInterfaceTest::receiveCommand, this);
            pub = nh.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states",100, false);
            gi_ptr = std::move(std::unique_ptr<GazeboInterface>(new GazeboInterface));
            aspinner =  std::move(std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1)));
            server = nh.advertiseService("get_action",&GazeboInterfaceTest::getAction, this);
            
        }
        virtual void TestBody(){}
        void setAction(double action) {
            this->action = action;
            t = std::thread(&GazeboInterfaceTest::actionResult,this, 8);
        
        }
        bool getAction(rl_msgs::GetAction::Request& req, rl_msgs::GetAction::Response& res) {
            res.action = this->action;
            return true;
        }

        void waitForCommandPublisher() {
            while (ros::ok() && sub.getNumPublishers() < 1) {
                ROS_ERROR("[GazeboInterfaceTest:: waitForCommandPublisher] publishers: %d",
                 sub.getNumPublishers());
                ros::Duration(0.1).sleep();
            }
        }
        void waitForTransitionPublisher() {
            while (ros::ok() && sub_t.getNumPublishers() < 1) {
                ROS_ERROR("[GazeboInterfaceTest:: waitForTransitionPublisher] publishers: %d",
                 sub_t.getNumPublishers());
                ros::Duration(0.1).sleep();
            }
        }
        void waitForModelSubscribers() {
            while (ros::ok() && pub.getNumSubscribers() < 1) {
                ROS_ERROR("[GazeboInterfaceTest:: waitForStateActionSubscribers] publishers: %d",
                 pub.getNumSubscribers());
                ros::Duration(0.1).sleep();
            }
        }
        void actionResult(double state) {
            action_result = this->gi_ptr->receiveAction(state);
        }
        bool createModelState(gazebo_msgs::GetModelState::Request &req,
            gazebo_msgs::GetModelState::Response &res) {
            ROS_ERROR("[GazeboInterfaceTest::createModelState] called");
            res.header.frame_id = "world";
            res.header.stamp = ros::Time::now();
            res.twist.linear.x =res.twist.linear.y =res.twist.linear.z = 0; 
            res.twist.angular.x = res.twist.angular.y = 0;
            res.twist.angular.z = -am;
            getter_service_called = true;

            return true;
        }

        void sendOdom(const geometry_msgs::Twist& cmd_vel) {
            gazebo_msgs::ModelStates ms;
            geometry_msgs::Pose pose;
            geometry_msgs::Twist twist;
            pose.position.x = 0; 
            pose.position.z =pose.position.y = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(0);
            ms.name.push_back(std::string("crumb"));
            ms.pose.push_back(pose);
            ms.twist.push_back(cmd_vel);
            pub.publish(ms);

            
        }
        void receiveCommand(const geometry_msgs::Twist& cmd_vel) {
            started_receiving = started_receiving || abs(cmd_vel.linear.x) == lm || abs(cmd_vel.angular.z) == am;
            ended_receiving = gi_ptr->isActionFinished(cmd_vel);
            sendOdom(cmd_vel);
        }

        void receiveTransition(const rl_msgs::TransitionReward& transition) {
            transition_received = true;
            this->transition.header.frame_id = transition.header.frame_id;
            this->transition.header.stamp = transition.header.stamp;
            this->transition.state = transition.state;
            this->transition.action = transition.action;
            this->transition.next_state = transition.next_state;
            this->transition.reward = transition.reward;
            this->transition.time_elapsed = transition.time_elapsed;
        }

    };

    TEST(GazeboInterfaceTest, shouldANilCommandBeReceivedTrueMustBeReturned) {
        GazeboInterfaceTest git_t;
        geometry_msgs::Twist cmd_vel, cmd_vel2, cmd_vel3, cmd_vel4;
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0001;
        cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0001;
        EXPECT_TRUE(git_t.gi_ptr->isActionFinished(cmd_vel));
        cmd_vel2.linear.x = cmd_vel2.linear.y = cmd_vel2.linear.z = -0.0002;
        cmd_vel2.angular.x = cmd_vel2.angular.y = cmd_vel2.angular.z = 0.0007;
        EXPECT_TRUE(git_t.gi_ptr->isActionFinished(cmd_vel2));
        cmd_vel3.linear.x = cmd_vel3.linear.y = cmd_vel3.linear.z = 0.003;
        cmd_vel3.angular.x = cmd_vel3.angular.y = cmd_vel3.angular.z = -0.0001;
        EXPECT_TRUE(git_t.gi_ptr->isActionFinished(cmd_vel3));
        cmd_vel4.linear.x = cmd_vel4.linear.y = cmd_vel4.linear.z = -0.0004;
        cmd_vel4.angular.x = cmd_vel4.angular.y = cmd_vel4.angular.z = -0.0004;
        EXPECT_TRUE(git_t.gi_ptr->isActionFinished(cmd_vel4));
        
    }

    TEST(GazeboInterfaceTest, shouldANonNilCommandBeReceivedFalseMustBeReturned) {
        GazeboInterfaceTest git_t;
        geometry_msgs::Twist cmd_vel, cmd_vel2;
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0001;
        cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 1;
        EXPECT_FALSE(git_t.gi_ptr->isActionFinished(cmd_vel));
        cmd_vel2.linear.x = cmd_vel2.linear.y = cmd_vel2.linear.z = -89;
        cmd_vel2.angular.x = cmd_vel2.angular.y = cmd_vel2.angular.z = 0.0007;
        EXPECT_FALSE(git_t.gi_ptr->isActionFinished(cmd_vel2));
        
        
    }

    TEST(GazeboInterfaceTest, shouldAnAction0BeIssuedCommandMustBeSent) {
        GazeboInterfaceTest git_t;
        const double action = 0;
        double next_state;
        git_t.waitForCommandPublisher();
        git_t.waitForModelSubscribers();
        git_t.aspinner->start();
        git_t.gi_ptr->sendCommand(action, next_state);
        git_t.aspinner->stop();
        EXPECT_TRUE(git_t.started_receiving);
        EXPECT_TRUE(git_t.ended_receiving);
    }
    
    TEST(GazeboInterfaceTest, shouldAnAction1BeIssuedCommandMustBeSent) {
        GazeboInterfaceTest git_t;
        const double action = 1;
        double next_state;
        git_t.waitForCommandPublisher();
        git_t.waitForModelSubscribers();
        git_t.aspinner->start();
        git_t.gi_ptr->sendCommand(action, next_state);
        git_t.aspinner->stop();
        EXPECT_TRUE(git_t.started_receiving);
        EXPECT_TRUE(git_t.ended_receiving);
    }
    TEST(GazeboInterfaceTest, shouldAnAction2BeIssuedCommandMustBeSent) {
        GazeboInterfaceTest git_t;
        const double action = 2;
        double next_state;
        git_t.waitForCommandPublisher();
        git_t.waitForModelSubscribers();
        git_t.aspinner->start();
        git_t.gi_ptr->sendCommand(action, next_state);
        git_t.aspinner->stop();
        EXPECT_TRUE(git_t.started_receiving);
        EXPECT_TRUE(git_t.ended_receiving);
    }
    TEST(GazeboInterfaceTest, shouldAnAction3BeIssuedCommandMustBeSent) {
        GazeboInterfaceTest git_t;
        const double action = 3;
        double next_state;
        git_t.waitForCommandPublisher();
        git_t.waitForModelSubscribers();
        git_t.aspinner->start();
        git_t.gi_ptr->sendCommand(action, next_state);
        git_t.aspinner->stop();
        EXPECT_TRUE(git_t.started_receiving);
        EXPECT_TRUE(git_t.ended_receiving);
    }

    TEST(GazeboInterfaceTest, receiveActionShouldReturnAction0CreatedByTheService) {
        GazeboInterfaceTest git_t;
        double expected = 0;
        git_t.setAction(expected);
        ros::Time start = ros::Time::now();
        do {
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < 0.1);
        git_t.t.join();

        EXPECT_EQ(expected, git_t.action_result);
    }
    
    TEST(GazeboInterfaceTest, receiveActionShouldReturnAction1CreatedByTheService) {
        GazeboInterfaceTest git_t;
        double expected = 1;
        git_t.setAction(expected);
        ros::Time start = ros::Time::now();
        do {
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < 0.1);
        git_t.t.join();

        EXPECT_EQ(expected, git_t.action_result);
    }

    TEST(GazeboInterfaceTest, receiveActionShouldReturnAction2CreatedByTheService) {
        GazeboInterfaceTest git_t;
        double expected = 2;
        git_t.setAction(expected);
        ros::Time start = ros::Time::now();
        do {
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < 0.1);
        git_t.t.join();

        EXPECT_EQ(expected, git_t.action_result);
    }

    TEST(GazeboInterfaceTest, receiveActionShouldReturnAction3CreatedByTheService) {
        GazeboInterfaceTest git_t;
        double expected = 3;
        git_t.setAction(expected);
        ros::Time start = ros::Time::now();
        do {
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < 0.1);
        git_t.t.join();

        EXPECT_EQ(expected, git_t.action_result);
    }
    TEST(GazeboInterfaceTest,transitionShouldBeReceivedCorrectly) {
        GazeboInterfaceTest git_t;
        double state = 8, action=0, next_state=3, reward = NAN, time_elapsed = 1.14;
        git_t.gi_ptr->sendTransition(state, action, next_state, time_elapsed);
        ros::Time start = ros::Time::now();
        do {
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        } while (ros::ok() && (ros::Time::now() - start).toSec() < 0.1);
        EXPECT_EQ(state, git_t.transition.state);
        EXPECT_EQ(action, git_t.transition.action);
        EXPECT_EQ(next_state, git_t.transition.next_state);
        EXPECT_TRUE(isnan(git_t.transition.reward));
        EXPECT_EQ(time_elapsed, git_t.transition.time_elapsed);
        
    }
    

}