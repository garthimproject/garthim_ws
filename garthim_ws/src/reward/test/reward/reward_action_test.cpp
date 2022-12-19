#include <gtest/gtest.h>
#include <reward/reward_action_function.h>

namespace reward {
    class RewardActionTest : public testing::Test {
        public:
            RewardActionTest() {
            }
            virtual void TestBody(){}
    };
    TEST(RewardActionTest, TestShouldBeInitCorrectly) {
        RewardActionFunction raf;
        std::vector<std::vector<double>> vecs;
        raf.getRewardsPerActionVector(vecs);
        std::vector<std::vector<double>> results = {{200.0, -4.3}, {200.0, -0.1}, {77.7, -32.0}, {200.0, -4.3}, {54.0, -4.3}};
        ASSERT_EQ(results,vecs);

    }

    TEST(RewardActionTest, TestShouldFailIfMalformed) {
        RewardActionFunction raf;
        std::vector<double> vecs={200.0, -4.3, 200.0, -0.1, 77.7, -32.0, 200.0, -4.3, 54.0, -4.3};
        EXPECT_ANY_THROW(raf.readRewardMatrixFromParam(vecs, 2,4));
    }

    TEST(RewardActionTest, ShouldAction2BeSelectedWithState4RewardShouldBe77dot7) {
        RewardActionFunction raf;
        rl_msgs::State state, next_state;
        rl_msgs::RLVariable action;
        action.as_integer = {2};
        state.content.as_integer = {4};
        next_state.content.as_integer = {4};
        double rew = raf.getReward(state,action,next_state);
        double exp_rew = 77.7;
        EXPECT_NEAR(rew,exp_rew, 0.1);
    }
} // namespace reward
