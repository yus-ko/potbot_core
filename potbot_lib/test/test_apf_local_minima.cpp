#include <gtest/gtest.h>
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::path_planner;
using namespace potbot_lib::potential;

// ============================================================
// 局所解回避テスト: 仮想障害物 + 渦巻き力
// ============================================================

// シナリオ1: 障害物背後
// ゴールが障害物の真裏にあり、引力と斥力が釣り合うケース
TEST(APFLocalMinimaTest, BehindSingleObstacle)
{
    // 41x41, res=0.1, wa=1.0, wr=5.0, dtr=1.0, origin=(0,0)
    ArtificialPotentialField apf(41, 41, 0.1, 1.0, 5.0, 1.0, 0.0, 0.0);
    apf.setVortexAngle(M_PI / 4.0);
    apf.setRobot(-1.5, 0.0);
    apf.setGoal(1.5, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(6.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 3, 1);

    bool result = planner.createPathWithVirtualObstacle(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 2u);

    // 経路の終点がゴール付近であること
    double end_dist = sqrt(pow(path.back().position.x - 1.5, 2) +
                           pow(path.back().position.y - 0.0, 2));
    EXPECT_LT(end_dist, 0.5);
}

// シナリオ2: 狭路通過
// 2つの障害物に挟まれた通路をAPFで通過できるか
TEST(APFLocalMinimaTest, NarrowPassage)
{
    // 41x41, res=0.1, wa=1.0, wr=5.0, dtr=0.8, origin=(0,0)
    ArtificialPotentialField apf(41, 41, 0.1, 1.0, 5.0, 0.8, 0.0, 0.0);
    apf.setVortexAngle(M_PI / 4.0);
    apf.setRobot(-1.5, 0.0);
    apf.setGoal(1.5, 0.0);
    apf.setObstacle(0.0, 0.6);
    apf.setObstacle(0.0, -0.6);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(6.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 3, 1);

    bool result = planner.createPathWithVirtualObstacle(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 2u);

    double end_dist = sqrt(pow(path.back().position.x - 1.5, 2) +
                           pow(path.back().position.y - 0.0, 2));
    EXPECT_LT(end_dist, 0.5);
}

// シナリオ3: 複数障害物の谷間
// 3つの障害物で形成されるポテンシャルの谷からの脱出
TEST(APFLocalMinimaTest, ValleyBetweenObstacles)
{
    // 61x61, res=0.1, wa=1.0, wr=5.0, dtr=1.0, origin=(0,0)
    ArtificialPotentialField apf(61, 61, 0.1, 1.0, 5.0, 1.0, 0.0, 0.0);
    apf.setVortexAngle(M_PI / 4.0);
    apf.setRobot(-2.0, 0.0);
    apf.setGoal(2.0, 0.0);
    apf.setObstacle(-0.5, 0.5);
    apf.setObstacle(0.0, 0.0);
    apf.setObstacle(0.5, -0.5);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(8.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 3, 1);

    bool result = planner.createPathWithVirtualObstacle(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 2u);

    double end_dist = sqrt(pow(path.back().position.x - 2.0, 2) +
                           pow(path.back().position.y - 0.0, 2));
    EXPECT_LT(end_dist, 0.5);

    // 経路長がユークリッド距離より長いこと（回り込みの検証）
    double path_length = 0;
    for (size_t i = 1; i < path.size(); i++)
    {
        path_length += sqrt(pow(path[i].position.x - path[i-1].position.x, 2) +
                            pow(path[i].position.y - path[i-1].position.y, 2));
    }
    double euclidean = sqrt(pow(2.0 - (-2.0), 2));
    EXPECT_GT(path_length, euclidean);
}

// 渦巻き力なし(vortex_angle=0)でもクラッシュしないことを確認
TEST(APFLocalMinimaTest, ZeroVortexAngleNoCrash)
{
    ArtificialPotentialField apf(41, 41, 0.1, 1.0, 5.0, 1.0, 0.0, 0.0);
    apf.setVortexAngle(0.0);
    apf.setRobot(-1.5, 0.0);
    apf.setGoal(1.5, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(6.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 3, 1);

    // クラッシュしないことが重要。成功/失敗は問わない
    planner.createPathWithVirtualObstacle(0.0);
    SUCCEED();
}

// createPath()ディスパッチが正しく動作することを確認
TEST(APFLocalMinimaTest, CreatePathDispatchVirtualObstacle)
{
    ArtificialPotentialField apf(41, 41, 0.1, 1.0, 5.0, 1.0, 0.0, 0.0);
    apf.setVortexAngle(M_PI / 4.0);
    apf.setRobot(-1.5, 0.0);
    apf.setGoal(1.5, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    planner.setParams(6.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 3, 1);

    // 障害物なしの場合、APF勾配降下のみでゴールに到達
    bool result = planner.createPath(0.0);
    EXPECT_TRUE(result);

    std::vector<Pose> path;
    planner.getPath(path);
    EXPECT_GT(path.size(), 0u);
}

// Dijkstraフォールバックが動作することを確認
TEST(APFLocalMinimaTest, DijkstraFallback)
{
    ArtificialPotentialField apf(41, 41, 0.1, 1.0, 5.0, 1.0, 0.0, 0.0);
    apf.setVortexAngle(M_PI / 4.0);
    apf.setRobot(-1.5, 0.0);
    apf.setGoal(1.5, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    APFPathPlanner planner(&apf);
    // max_escape_attempts=0にして仮想障害物を使わせない → Dijkstra fallback
    planner.setParams(6.0, 1, 0.0, 1.0,
                      "virtual_obstacle_vortex", 0, 1);

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
