#include <gtest/gtest.h>
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::path_planner;
using namespace potbot_lib::potential;

// コンストラクタ引数順序: (rows, cols, resolution, weight_attr, weight_rep, dtr, origin_x, origin_y)
// 11x11フィールド、res=1.0、origin=(0,0)の場合の座標範囲: x:[-5.5, 4.5], y:[-5.5, 4.5]
// 有効な座標範囲に収まるようにロボット・ゴール・障害物を設定する

// ============================================================
// APFPathPlanner 基本テスト
// ============================================================

TEST(APFPathPlannerTest, Constructor)
{
    ArtificialPotentialField apf(11, 11, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    APFPathPlanner planner(&apf);
    SUCCEED();
}

TEST(APFPathPlannerTest, SetParams)
{
    ArtificialPotentialField apf(11, 11, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    APFPathPlanner planner(&apf);
    planner.setParams(10.0, 2, 0.5, 0.5);
    SUCCEED();
}

// ============================================================
// createPath() テスト
// ============================================================

TEST(APFPathPlannerTest, CreatePathReturnsTrue)
{
    // 11x11, res=1.0, wa=1.0, wr=0.0, dtr=100.0, origin=(0,0)
    // 座標範囲: x:[-5.5, 4.5], y:[-5.5, 4.5]
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-4.0, 0.0);
    apf.setGoal(4.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    bool result = planner.createPath(0.0);
    EXPECT_TRUE(result);
}

TEST(APFPathPlannerTest, CreatePathProducesNonEmptyPath)
{
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-4.0, 0.0);
    apf.setGoal(4.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

TEST(APFPathPlannerTest, CreatePathStartsAtRobot)
{
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-3.0, 0.0);
    apf.setGoal(3.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 0u);

    // 経路の最初の点はロボット位置付近
    Point robot = apf.getRobot();
    double dist_start = std::hypot(
        path[0].position.x - robot.x,
        path[0].position.y - robot.y);
    EXPECT_LT(dist_start, 1.5);  // 1グリッド分以内
}

TEST(APFPathPlannerTest, CreatePathEndsNearGoal)
{
    // createPath()内に norm()>0.1 の距離チェックがあるため、
    // グリッド解像度を0.05m以下にする必要がある（対角距離 0.05*sqrt(2)≈0.071m < 0.1m）
    // 41x41, res=0.05m, origin=(0,0): x/y範囲 [-1.025, 1.025]
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 1u);  // 複数の経路点が生成される

    // 経路の最後の点はゴール付近（解像度0.05mの数グリッド以内）
    Point goal = apf.getGoal();
    double dist_end = std::hypot(
        path.back().position.x - goal.x,
        path.back().position.y - goal.y);
    EXPECT_LT(dist_end, 0.3);  // 0.3m以内（6グリッド相当）
}

TEST(APFPathPlannerTest, CreatePathWithObstacle)
{
    // 障害物がある場合も経路が生成される
    // 21x21, res=1.0, wa=1.0, wr=5.0, dtr=3.0, origin=(0,0)
    ArtificialPotentialField apf(21, 21, 1.0, 1.0, 5.0, 3.0, 0.0, 0.0);
    apf.setRobot(-8.0, 0.0);
    apf.setGoal(8.0, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(25.0, 1, 1.0, 0.0);
    bool result = planner.createPath(0.0);

    EXPECT_TRUE(result);
    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// ============================================================
// createPathWithWeight() テスト
// ============================================================

TEST(APFPathPlannerTest, CreatePathWithWeightReturnsTrue)
{
    // createPathWithWeightはIS_ROBOTフラグを使用
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-3.0, 0.0);   // IS_ROBOTフラグをセット
    apf.setGoal(3.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(10.0, 1, 0.5, 0.5);
    bool result = planner.createPathWithWeight(0.0);
    EXPECT_TRUE(result);
}

TEST(APFPathPlannerTest, CreatePathWithWeightProducesPath)
{
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-3.0, 0.0);
    apf.setGoal(3.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(10.0, 1, 0.5, 0.5);
    planner.createPathWithWeight(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// ============================================================
// bezier() テスト
// ============================================================

TEST(APFPathPlannerTest, BezierOnEmptyPathReturnsFalse)
{
    // createPath()を呼ぶ前（path_が空）はfalseを返す
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    APFPathPlanner planner(&apf);

    bool result = planner.bezier();
    EXPECT_FALSE(result);
}

TEST(APFPathPlannerTest, BezierAfterCreatePath)
{
    // 小さい解像度でcreatePath()が複数の経路点を生成し、bezier()を検証する
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);

    std::vector<Pose> path_before;
    planner.getPath(path_before);

    if (path_before.size() >= 2)
    {
        bool result = planner.bezier();
        EXPECT_TRUE(result);

        std::vector<Pose> path_after;
        planner.getPath(path_after);
        EXPECT_GT(path_after.size(), 0u);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
