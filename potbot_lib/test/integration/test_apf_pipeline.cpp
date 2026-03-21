// APFパイプライン統合テスト
// Navigation2の統合テストパターン（複数コンポーネントの協調動作）を踏襲
// APFPathPlanner → OptimalPathFollower → DiffDriveAgent の一連動作を検証する

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <potbot_lib/diff_drive_agent.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_lib/optimal_path_follower.hpp>

using namespace potbot_lib;
using namespace potbot_lib::controller;
using namespace potbot_lib::path_planner;
using namespace potbot_lib::potential;

// ============================================================
// テスト1: APFが経路を生成し、OPFがその経路を受け取り、DiffDriveが追従できるか
// ============================================================
TEST(APFPipelineTest, FullNavigationCycle)
{
    // 1. APFフィールド生成
    // 41x41, res=0.05m, wa=1.0, wr=0.0, dtr=10.0, origin=(0,0)
    // x/y範囲: [-1.025, 1.025]
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    // 2. APFPathPlannerで経路生成
    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    bool path_created = planner.createPath(0.0);
    ASSERT_TRUE(path_created);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 1u);

    // 3. OptimalPathFollowerに経路を設定
    OptimalPathFollower follower;
    follower.setLimit(-0.3, 0.3, -1.5, 1.5);
    follower.setOptimizationMethod("all_search");
    follower.setTimeIncrement(0.1);
    follower.setTimeEnd(1.0);
    follower.setLinearVelocityIncrement(0.1);
    follower.setAngularVelocityIncrement(0.5);
    follower.deltatime = 0.1;

    // ロボット初期位置を経路起点に合わせる
    follower.x = path[0].position.x;
    follower.y = path[0].position.y;
    follower.yaw = 0.0;
    follower.setTargetPath(path);

    // 4. DiffDriveAgent（OptimalPathFollower継承）でシミュレーション実行
    // 最大200ステップで追従シミュレーションを実行
    int max_steps = 200;
    bool reached = false;
    for (int i = 0; i < max_steps; i++)
    {
        if (follower.reachedTarget())
        {
            reached = true;
            break;
        }
        follower.calculateCommand();
        follower.update();
    }

    // 5. ゴールに近づいたかを検証（許容誤差0.5m）
    Point goal = apf.getGoal();
    double dist_to_goal = std::hypot(follower.x - goal.x, follower.y - goal.y);
    EXPECT_TRUE(reached);
    EXPECT_LT(dist_to_goal, 0.5);
}

// ============================================================
// テスト2: 障害物回避を含むフルパイプライン
// ============================================================
TEST(APFPipelineTest, NavigationWithObstacle)
{
    // 障害物がある場合のフルパイプライン
    // 41x41, res=0.05m, wa=1.0, wr=5.0, dtr=0.3, origin=(0,0)
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 5.0, 0.3, 0.0, 0.0);
    apf.setRobot(-0.8, 0.0);
    apf.setGoal(0.8, 0.0);
    // ロボットとゴールの中間に障害物を設置
    apf.setObstacle(0.0, 0.1);
    apf.createPotentialField();

    // APFPathPlannerで経路生成
    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    bool path_created = planner.createPath(0.0);
    ASSERT_TRUE(path_created);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 0u);

    // OptimalPathFollowerに経路を設定してシミュレーション実行
    OptimalPathFollower follower;
    follower.setLimit(-0.3, 0.3, -1.5, 1.5);
    follower.setOptimizationMethod("all_search");
    follower.setTimeIncrement(0.1);
    follower.setTimeEnd(1.0);
    follower.setLinearVelocityIncrement(0.1);
    follower.setAngularVelocityIncrement(0.5);
    follower.deltatime = 0.1;

    follower.x = path[0].position.x;
    follower.y = path[0].position.y;
    follower.yaw = 0.0;
    follower.setTargetPath(path);

    int max_steps = 200;
    for (int i = 0; i < max_steps; i++)
    {
        if (follower.reachedTarget())
        {
            break;
        }
        follower.calculateCommand();
        follower.update();
    }

    // 経路が生成され、追従コントローラが正常に動作すること
    // ゴール到達または十分ゴールに近づくことを検証（障害物回避により迂回するため許容誤差は広め）
    Point goal = apf.getGoal();
    double dist_to_goal = std::hypot(follower.x - goal.x, follower.y - goal.y);
    EXPECT_LT(dist_to_goal, 1.5);
}

// ============================================================
// テスト3: 複数回の経路再計画
// ============================================================
TEST(APFPipelineTest, MultipleReplanningCycles)
{
    // ゴールを変えながら複数回計画→追従を繰り返す
    // 各サイクルで経路生成と追従が正常に動作することを検証

    // サイクル1: ロボット(-0.5,0) → ゴール(0.5,0)
    {
        ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
        apf.setRobot(-0.5, 0.0);
        apf.setGoal(0.5, 0.0);
        apf.createPotentialField();

        APFPathPlanner planner(&apf);
        planner.setParams(3.0, 1, 1.0, 0.0);
        bool path_created = planner.createPath(0.0);
        ASSERT_TRUE(path_created);

        std::vector<Pose> path;
        planner.getPath(path);
        ASSERT_GT(path.size(), 0u);

        OptimalPathFollower follower;
        follower.setLimit(-0.3, 0.3, -1.5, 1.5);
        follower.setOptimizationMethod("all_search");
        follower.setTimeIncrement(0.1);
        follower.setTimeEnd(1.0);
        follower.setLinearVelocityIncrement(0.1);
        follower.setAngularVelocityIncrement(0.5);
        follower.deltatime = 0.1;

        follower.x = path[0].position.x;
        follower.y = path[0].position.y;
        follower.yaw = 0.0;
        follower.setTargetPath(path);

        bool reached = false;
        for (int i = 0; i < 200; i++)
        {
            if (follower.reachedTarget())
            {
                reached = true;
                break;
            }
            follower.calculateCommand();
            follower.update();
        }
        EXPECT_TRUE(reached);
    }

    // サイクル2: ロボット(0,-0.5) → ゴール(0,0.5)（Y軸方向）
    {
        ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
        apf.setRobot(0.0, -0.5);
        apf.setGoal(0.0, 0.5);
        apf.createPotentialField();

        APFPathPlanner planner(&apf);
        planner.setParams(3.0, 1, 1.0, 0.0);
        bool path_created = planner.createPath(0.0);
        ASSERT_TRUE(path_created);

        std::vector<Pose> path;
        planner.getPath(path);
        ASSERT_GT(path.size(), 0u);

        OptimalPathFollower follower;
        follower.setLimit(-0.3, 0.3, -1.5, 1.5);
        follower.setOptimizationMethod("all_search");
        follower.setTimeIncrement(0.1);
        follower.setTimeEnd(1.0);
        follower.setLinearVelocityIncrement(0.1);
        follower.setAngularVelocityIncrement(0.5);
        follower.deltatime = 0.1;

        follower.x = path[0].position.x;
        follower.y = path[0].position.y;
        follower.yaw = M_PI / 2.0;
        follower.setTargetPath(path);

        bool reached = false;
        for (int i = 0; i < 200; i++)
        {
            if (follower.reachedTarget())
            {
                reached = true;
                break;
            }
            follower.calculateCommand();
            follower.update();
        }
        EXPECT_TRUE(reached);
    }

    // サイクル3: 斜め方向 ロボット(-0.5,-0.5) → ゴール(0.5,0.5)
    {
        ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
        apf.setRobot(-0.5, -0.5);
        apf.setGoal(0.5, 0.5);
        apf.createPotentialField();

        APFPathPlanner planner(&apf);
        planner.setParams(3.0, 1, 1.0, 0.0);
        bool path_created = planner.createPath(0.0);
        ASSERT_TRUE(path_created);

        std::vector<Pose> path;
        planner.getPath(path);
        ASSERT_GT(path.size(), 0u);

        OptimalPathFollower follower;
        follower.setLimit(-0.3, 0.3, -1.5, 1.5);
        follower.setOptimizationMethod("all_search");
        follower.setTimeIncrement(0.1);
        follower.setTimeEnd(1.0);
        follower.setLinearVelocityIncrement(0.1);
        follower.setAngularVelocityIncrement(0.5);
        follower.deltatime = 0.1;

        follower.x = path[0].position.x;
        follower.y = path[0].position.y;
        follower.yaw = M_PI / 4.0;
        follower.setTargetPath(path);

        int max_steps = 200;
        for (int i = 0; i < max_steps; i++)
        {
            if (follower.reachedTarget())
            {
                break;
            }
            follower.calculateCommand();
            follower.update();
        }

        // 斜め方向でもゴールに近づくこと
        Point goal = apf.getGoal();
        double dist_to_goal = std::hypot(follower.x - goal.x, follower.y - goal.y);
        EXPECT_LT(dist_to_goal, 1.0);
    }
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
