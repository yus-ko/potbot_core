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

// ============================================================
// パス品質ベンチマーク（障害物なし）
// ============================================================

TEST(APFPathPlannerTest, PathLengthNearEuclideanDistance)
{
    // 障害物なしで経路長がユークリッド距離の1.5倍以内であること
    // 41x41, res=0.05, robot=(-0.9,0), goal=(0.9,0), ユークリッド距離=1.8m
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 1u);

    // 経路長を計算
    double path_length = 0.0;
    for (size_t i = 1; i < path.size(); i++)
    {
        double dx = path[i].position.x - path[i-1].position.x;
        double dy = path[i].position.y - path[i-1].position.y;
        path_length += std::hypot(dx, dy);
    }
    double euclidean_dist = 1.8;
    // 経路長はユークリッド距離の1.5倍以内であること
    EXPECT_LT(path_length, euclidean_dist * 1.5);
}

// ============================================================
// createPath() の冪等性テスト
// ============================================================

TEST(APFPathPlannerTest, CreatePathIdempotent)
{
    // createPath()を2回連続で呼んでも2回目の結果が有効であること
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);
    bool result2 = planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_TRUE(result2);
    EXPECT_GT(path.size(), 0u);
}

// ============================================================
// Robot == Goal でクラッシュなし
// ============================================================

TEST(APFPathPlannerTest, RobotEqualsGoalNocrash)
{
    // Robot と Goal が同じ座標(0,0)でもcreatePathがクラッシュしないこと
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(0.0, 0.0);
    apf.setGoal(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    EXPECT_NO_THROW({
        planner.createPath(0.0);
    });
}

// ============================================================
// Y軸方向の経路生成
// ============================================================

TEST(APFPathPlannerTest, CreatePathYDirection)
{
    // Robot=(0,-0.9), Goal=(0,0.9) でY軸方向の経路が生成されること
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(0.0, -0.9);
    apf.setGoal(0.0, 0.9);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// ============================================================
// 経路が単調にゴールに近づくかの確認（障害物なし）
// ============================================================

TEST(APFPathPlannerTest, PathMonotonicallyApproachesGoal)
{
    // 経路の大半の点がゴールに単調に近づいていること
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);
    planner.createPath(0.0);

    std::vector<Pose> path;
    planner.getPath(path);
    ASSERT_GT(path.size(), 2u);

    Point goal = apf.getGoal();
    int monotone_count = 0;
    for (size_t i = 1; i < path.size(); i++)
    {
        double d_prev = std::hypot(path[i-1].position.x - goal.x, path[i-1].position.y - goal.y);
        double d_curr = std::hypot(path[i].position.x - goal.x, path[i].position.y - goal.y);
        if (d_curr < d_prev) monotone_count++;
    }
    // 大半(70%以上)の点で単調減少すること
    EXPECT_GT(monotone_count, static_cast<int>(path.size() * 0.7));
}

// ============================================================
// バグ修正検証テスト（M7 T-003）
// ============================================================

// CreatePathDoesNotLoopWithFarGoal:
//   ゴールが遠い場合に path が100点未満で終了することを確認（バグ5の検証）
TEST(APFPathPlannerTest, CreatePathDoesNotLoopWithFarGoal)
{
    // 61x61, res=0.1m, robot=(-2.8,0), goal=(2.8,0)で長い経路を試みる
    // ループが発生しなければpath.size()は100未満で終了するはず
    ArtificialPotentialField apf(61, 61, 0.1, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-2.8, 0.0);
    apf.setGoal(2.8, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(10.0, 1, 1.0, 0.0);
    bool result = planner.createPath(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
    // ループが発生すると path.size() == 101 (>100でbreak) になる。
    // バグ修正後は15ステップ進捗なしでbreakするため100未満のはず
    EXPECT_LT(path.size(), 101u);
}

// CreatePathHandlesEmptyEdges:
//   障害物がない場合（エッジが空）でもcreatePath()がクラッシュしないことを確認（バグ2の検証）
TEST(APFPathPlannerTest, CreatePathHandlesEmptyEdges)
{
    // 障害物なし（斥力フィールドなし）で斥力エッジが空になる
    ArtificialPotentialField apf(21, 21, 0.1, 1.0, 0.0, 0.3, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(5.0, 1, 1.0, 0.0);

    // 障害物なし（エッジなし）でもクラッシュしないこと
    EXPECT_NO_THROW({
        planner.createPath(0.0);
    });

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// CreatePathDuplicateDetection:
//   path_.size() < 2 のときに重複検出が安全であることを確認（バグ1の検証）
TEST(APFPathPlannerTest, CreatePathDuplicateDetection)
{
    // Robot と Goal が隣接するグリッドの場合、path_が1点のときに重複検出が呼ばれる可能性がある
    // バグ修正後は path_.size() >= 2 のガードがあるためクラッシュしない
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setRobot(-1.0, 0.0);
    apf.setGoal(1.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(5.0, 1, 1.0, 0.0);

    // path_.size() < 2 のときでもクラッシュしないこと
    EXPECT_NO_THROW({
        planner.createPath(0.0);
    });
}

// ============================================================
// createPathDijkstra() テスト（M8 T-001）
// ============================================================

// CreatePathDijkstraBasic:
//   障害物なし・ゴールに向かう基本ケースでDijkstraが経路を生成することを確認
TEST(APFPathPlannerTest, CreatePathDijkstraBasic)
{
    // 41x41, res=0.05m, robot=(-0.9,0), goal=(0.9,0), 障害物なし
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);

    bool result = planner.createPathDijkstra(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// CreatePathDijkstraWithObstacle:
//   障害物がある場合もDijkstraがクラッシュしないことを確認
TEST(APFPathPlannerTest, CreatePathDijkstraWithObstacle)
{
    // 21x21, res=1.0, robot=(-8,0), goal=(8,0), obstacle=(0,0)
    ArtificialPotentialField apf(21, 21, 1.0, 1.0, 5.0, 3.0, 0.0, 0.0);
    apf.setRobot(-8.0, 0.0);
    apf.setGoal(8.0, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(25.0, 1, 1.0, 0.0);

    // 障害物があってもクラッシュしないこと
    EXPECT_NO_THROW({
        planner.createPathDijkstra(0.0);
    });
}

// CreatePathProducesValidPath:
//   createPath()が障害物なし環境でパスを生成できることを確認
TEST(APFPathPlannerTest, CreatePathProducesValidPath)
{
    // 41x41, res=0.05m, robot=(-0.9,0), goal=(0.9,0), 障害物なし
    ArtificialPotentialField apf(41, 41, 0.05, 1.0, 0.0, 10.0, 0.0, 0.0);
    apf.setRobot(-0.9, 0.0);
    apf.setGoal(0.9, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(3.0, 1, 1.0, 0.0);

    // createPath()は経路を生成する（内部で適切な手法にディスパッチ）
    bool result = planner.createPath(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
