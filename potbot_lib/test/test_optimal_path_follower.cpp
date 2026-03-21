#include <gtest/gtest.h>
#include <potbot_lib/optimal_path_follower.hpp>

using namespace potbot_lib;
using namespace potbot_lib::controller;

// ============================================================
// ヘルパー関数: 始点(0,0)からgoal(goal_x, goal_y)へのPoseリストを生成する
// ============================================================
static std::vector<potbot_lib::Pose> make_path(double goal_x, double goal_y, int n = 10)
{
    std::vector<potbot_lib::Pose> path;
    for (int i = 0; i <= n; i++)
    {
        double t = static_cast<double>(i) / n;
        path.push_back(potbot_lib::Pose(goal_x * t, goal_y * t, 0));
    }
    return path;
}

// ============================================================
// 1. Constructor — デフォルト構築が成功すること
// ============================================================
TEST(OptimalPathFollowerTest, Constructor)
{
    OptimalPathFollower follower;
    SUCCEED();
}

// ============================================================
// 2. InitialState — 初期状態で v=0, omega=0, x=0, y=0 であること
// ============================================================
TEST(OptimalPathFollowerTest, InitialState)
{
    OptimalPathFollower follower;
    EXPECT_NEAR(follower.v,     0.0, 1e-9);
    EXPECT_NEAR(follower.omega, 0.0, 1e-9);
    EXPECT_NEAR(follower.x,     0.0, 1e-9);
    EXPECT_NEAR(follower.y,     0.0, 1e-9);
}

// ============================================================
// 3. EmptyPathReachedTarget — setTargetPath()未呼び出し → reachedTarget()=true
// ============================================================
TEST(OptimalPathFollowerTest, EmptyPathReachedTarget)
{
    OptimalPathFollower follower;
    EXPECT_TRUE(follower.reachedTarget());
}

// ============================================================
// 4. SetTargetPath — Poseリストをセット後、reachedTarget()が適切な値を返す
//    ロボットは(0,0)、ゴールは(5,0)なので距離が大きく、未到達のはず
// ============================================================
TEST(OptimalPathFollowerTest, SetTargetPath)
{
    OptimalPathFollower follower;
    auto path = make_path(5.0, 0.0);
    follower.setTargetPath(path);
    EXPECT_FALSE(follower.reachedTarget());
}

// ============================================================
// 5. SetMarginAndLimit — setMargin/setLimitを呼んでも例外が発生しないこと
// ============================================================
TEST(OptimalPathFollowerTest, SetMarginAndLimit)
{
    OptimalPathFollower follower;
    EXPECT_NO_THROW(follower.setMargin(0.2, 0.05));
    EXPECT_NO_THROW(follower.setLimit(-0.5, 0.5, -1.5, 1.5));
}

// ============================================================
// 6. SettersNoThrow — 各種セッターを呼んでも例外が発生しないこと
// ============================================================
TEST(OptimalPathFollowerTest, SettersNoThrow)
{
    OptimalPathFollower follower;
    EXPECT_NO_THROW(follower.setOptimizationMethod("all_search"));
    EXPECT_NO_THROW(follower.setOptimizationMethod("gradient"));
    EXPECT_NO_THROW(follower.setTimeIncrement(0.05));
    EXPECT_NO_THROW(follower.setTimeEnd(2.0));
    EXPECT_NO_THROW(follower.setLinearVelocityIncrement(0.05));
    EXPECT_NO_THROW(follower.setAngularVelocityIncrement(0.1));
    EXPECT_NO_THROW(follower.setIterationMax(50));
    EXPECT_NO_THROW(follower.setLearningRate(0.01));
}

// ============================================================
// 7. CalculateCommandAllSearch — "all_search"法でcalculateCommand()後にv, omegaが制限範囲内
// ============================================================
TEST(OptimalPathFollowerTest, CalculateCommandAllSearch)
{
    OptimalPathFollower follower;
    double v_min = -0.2, v_max = 0.2;
    double omega_min = -1.0, omega_max = 1.0;
    follower.setLimit(v_min, v_max, omega_min, omega_max);
    follower.setOptimizationMethod("all_search");
    follower.setTimeIncrement(0.1);
    follower.setTimeEnd(0.5);
    follower.setLinearVelocityIncrement(0.1);
    follower.setAngularVelocityIncrement(0.5);

    auto path = make_path(1.0, 0.0);
    follower.setTargetPath(path);

    EXPECT_NO_THROW(follower.calculateCommand());

    EXPECT_GE(follower.v,     v_min);
    EXPECT_LE(follower.v,     v_max);
    EXPECT_GE(follower.omega, omega_min);
    EXPECT_LE(follower.omega, omega_max);
}

// ============================================================
// 8. CalculateCommandGradient — "gradient"法でcalculateCommand()後にv, omegaが制限範囲内
// ============================================================
TEST(OptimalPathFollowerTest, CalculateCommandGradient)
{
    OptimalPathFollower follower;
    double v_min = -0.2, v_max = 0.2;
    double omega_min = -1.0, omega_max = 1.0;
    follower.setLimit(v_min, v_max, omega_min, omega_max);
    follower.setOptimizationMethod("gradient");
    follower.setTimeIncrement(0.1);
    follower.setTimeEnd(0.5);
    follower.setIterationMax(20);
    follower.setLearningRate(0.01);

    auto path = make_path(1.0, 0.0);
    follower.setTargetPath(path);

    EXPECT_NO_THROW(follower.calculateCommand());

    EXPECT_GE(follower.v,     v_min);
    EXPECT_LE(follower.v,     v_max);
    EXPECT_GE(follower.omega, omega_min);
    EXPECT_LE(follower.omega, omega_max);
}

// ============================================================
// 9. GettersAfterCalculate — calculateCommand()後に各ゲッターが正常動作すること
// ============================================================
TEST(OptimalPathFollowerTest, GettersAfterCalculate)
{
    OptimalPathFollower follower;
    double v_min = -0.2, v_max = 0.2;
    double omega_min = -1.0, omega_max = 1.0;
    follower.setLimit(v_min, v_max, omega_min, omega_max);
    follower.setOptimizationMethod("all_search");
    follower.setTimeIncrement(0.1);
    follower.setTimeEnd(0.5);
    follower.setLinearVelocityIncrement(0.1);
    follower.setAngularVelocityIncrement(0.5);

    auto path = make_path(1.0, 0.0);
    follower.setTargetPath(path);
    follower.calculateCommand();

    std::vector<plan> plans;
    EXPECT_NO_THROW(follower.getPlans(plans));
    EXPECT_GT(plans.size(), 0u);

    std::vector<Eigen::Vector2d> split_path;
    EXPECT_NO_THROW(follower.getSplitPath(split_path));

    std::vector<Eigen::Vector2d> best_path;
    EXPECT_NO_THROW(follower.getBestPath(best_path));

    plan best_plan;
    EXPECT_NO_THROW(follower.getBestPlan(best_plan));

    double cmd_v = 0, cmd_omega = 0;
    EXPECT_NO_THROW(follower.getBestCmd(cmd_v, cmd_omega));
    EXPECT_GE(cmd_v,     v_min);
    EXPECT_LE(cmd_v,     v_max);
    EXPECT_GE(cmd_omega, omega_min);
    EXPECT_LE(cmd_omega, omega_max);
}

// ============================================================
// 10. ReachedTargetWhenClose — ロボットがゴール近傍にいるとき reachedTarget()=true
// ============================================================
TEST(OptimalPathFollowerTest, ReachedTargetWhenClose)
{
    OptimalPathFollower follower;
    // デフォルトのstop_margin_distance_(0.03m)より近い距離(0.01m)にゴールを置く
    auto path = make_path(0.01, 0.0);
    follower.setTargetPath(path);
    EXPECT_TRUE(follower.reachedTarget());
}

// ============================================================
// 11. NotReachedTargetWhenFar — ロボットがゴールから遠いとき reachedTarget()=false
// ============================================================
TEST(OptimalPathFollowerTest, NotReachedTargetWhenFar)
{
    OptimalPathFollower follower;
    auto path = make_path(5.0, 0.0);
    follower.setTargetPath(path);
    EXPECT_FALSE(follower.reachedTarget());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
