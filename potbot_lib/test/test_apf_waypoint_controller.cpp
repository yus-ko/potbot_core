#include <gtest/gtest.h>
#include "potbot_lib/apf_waypoint_controller.hpp"

using namespace potbot_lib;
using namespace potbot_lib::controller;

// ============================================================
// ヘルパー関数: 始点(0,0)からgoal(goal_x, goal_y)へのPoseリストを生成する
// ============================================================
static std::vector<potbot_lib::Pose> make_path(double goal_x, double goal_y, int n = 10)
{
    std::vector<potbot_lib::Pose> path;
    for (int i = 0; i <= n; i++) {
        double t = static_cast<double>(i) / n;
        path.push_back(potbot_lib::Pose(goal_x * t, goal_y * t, 0));
    }
    return path;
}

// ============================================================
// 1. Constructor — デフォルト構築が成功すること
// ============================================================
TEST(ApfWaypointControllerTest, Constructor)
{
    ApfWaypointController ctrl;
    SUCCEED();
}

// ============================================================
// 2. EmptyPathIsPathEmpty — setGlobalPath()未呼び出しは isPathEmpty()=true
// ============================================================
TEST(ApfWaypointControllerTest, EmptyPathIsPathEmpty)
{
    ApfWaypointController ctrl;
    EXPECT_TRUE(ctrl.isPathEmpty());
}

// ============================================================
// 3. SetGlobalPath — パスをセット後、isPathEmpty()=false
// ============================================================
TEST(ApfWaypointControllerTest, SetGlobalPath)
{
    ApfWaypointController ctrl;
    auto path = make_path(1.0, 0.0);
    ctrl.setGlobalPath(path);
    EXPECT_FALSE(ctrl.isPathEmpty());
}

// ============================================================
// 4. WaypointAdvances — ロボットをwaypointの近傍に置いてcomputeCommand後にindexが進む
// ============================================================
TEST(ApfWaypointControllerTest, WaypointAdvances)
{
    ApfWaypointController ctrl;
    ctrl.setParams(1.0, 0.0, 0.5, 0.5, 2.0, 0.3, 1.5, 0.2, 0.05);

    // 5点のパス: (0,0),(0.5,0),(1.0,0),(1.5,0),(2.0,0)
    std::vector<Pose> path;
    path.push_back(Pose(0.0, 0.0, 0.0));
    path.push_back(Pose(0.5, 0.0, 0.0));
    path.push_back(Pose(1.0, 0.0, 0.0));
    path.push_back(Pose(1.5, 0.0, 0.0));
    path.push_back(Pose(2.0, 0.0, 0.0));
    ctrl.setGlobalPath(path);

    // ロボットを最初のwaypointの近傍(tolerance=0.2より近い)に置く
    ctrl.x   = 0.05;
    ctrl.y   = 0.0;
    ctrl.yaw = 0.0;

    size_t idx_before = ctrl.getCurrentWaypointIndex();
    ctrl.computeCommand();
    size_t idx_after = ctrl.getCurrentWaypointIndex();

    EXPECT_GT(idx_after, idx_before);
}

// ============================================================
// 5. ReachedGoal — ロボットを最終waypoint近傍に置いてreachedGoal()=true
// ============================================================
TEST(ApfWaypointControllerTest, ReachedGoal)
{
    ApfWaypointController ctrl;
    ctrl.setParams(1.0, 0.0, 0.5, 0.5, 2.0, 0.3, 1.5, 0.2, 0.05);

    auto path = make_path(2.0, 0.0, 4);
    ctrl.setGlobalPath(path);

    // ゴール(2.0, 0.0)の近傍(0.02m)にロボットを置く
    ctrl.x   = 2.0 - 0.02;
    ctrl.y   = 0.0;
    ctrl.yaw = 0.0;

    EXPECT_TRUE(ctrl.reachedGoal());
}

// ============================================================
// 6. VelocityWithinLimits — v ∈ [0, v_max], |omega| ∈ [0, omega_max]
// ============================================================
TEST(ApfWaypointControllerTest, VelocityWithinLimits)
{
    ApfWaypointController ctrl;
    double v_max     = 0.3;
    double omega_max = 1.5;
    ctrl.setParams(1.0, 0.1, 0.5, 0.5, 2.0, v_max, omega_max, 0.2, 0.05);

    auto path = make_path(2.0, 0.0);
    ctrl.setGlobalPath(path);
    ctrl.x   = 0.0;
    ctrl.y   = 0.0;
    ctrl.yaw = 0.0;

    ctrl.computeCommand();

    EXPECT_GE(ctrl.v,     0.0);
    EXPECT_LE(ctrl.v,     v_max);
    EXPECT_GE(ctrl.omega, -omega_max);
    EXPECT_LE(ctrl.omega,  omega_max);
}

// ============================================================
// 7. ForwardTargetProducesPositiveV — ゴールが正面にある場合 v > 0
// ============================================================
TEST(ApfWaypointControllerTest, ForwardTargetProducesPositiveV)
{
    ApfWaypointController ctrl;
    ctrl.setParams(1.0, 0.0, 0.5, 0.5, 2.0, 0.3, 1.5, 0.2, 0.05);

    // ゴールはロボットの正面(+x方向)
    std::vector<Pose> path;
    path.push_back(Pose(2.0, 0.0, 0.0));
    ctrl.setGlobalPath(path);

    ctrl.x   = 0.0;
    ctrl.y   = 0.0;
    ctrl.yaw = 0.0; // 正面を向いている

    ctrl.computeCommand();

    EXPECT_GT(ctrl.v, 0.0);
}

// ============================================================
// 8. ObstacleRepulsionChangesOutput — 障害物ありとなしでcomputeCommand()の出力が異なる
// ============================================================
TEST(ApfWaypointControllerTest, ObstacleRepulsionChangesOutput)
{
    auto run = [](bool with_obstacle) -> double {
        ApfWaypointController ctrl;
        ctrl.setParams(1.0, 2.0, 0.5, 0.5, 2.0, 0.3, 1.5, 0.2, 0.05);

        std::vector<Pose> path;
        path.push_back(Pose(2.0, 0.0, 0.0));
        ctrl.setGlobalPath(path);

        ctrl.x   = 0.0;
        ctrl.y   = 0.0;
        ctrl.yaw = 0.0;

        if (with_obstacle) {
            std::vector<Point> obs;
            obs.push_back(Point{0.3, 0.0, 0.0}); // 進行方向正面の近くに障害物
            ctrl.setObstacles(obs);
        }

        ctrl.computeCommand();
        return ctrl.omega; // 障害物があれば回避のためomegaが変わるはず
    };

    double omega_no_obs   = run(false);
    double omega_with_obs = run(true);

    // 障害物があるとomegaが変化することを確認
    EXPECT_NE(omega_no_obs, omega_with_obs);
}

// ============================================================
// 9. ConvergenceSimulation — 500ステップ以内にreachedGoal()になること
// ============================================================
TEST(ApfWaypointControllerTest, ConvergenceSimulation)
{
    ApfWaypointController ctrl;
    ctrl.setParams(1.0, 0.0, 0.5, 0.5, 2.0, 0.3, 1.5, 0.2, 0.05);

    // 単純な直線経路 robot=(0,0), goal=(2,0)
    std::vector<Pose> path;
    for (int i = 1; i <= 5; i++) {
        path.push_back(Pose(0.4 * i, 0.0, 0.0));
    }
    ctrl.setGlobalPath(path);

    ctrl.x       = 0.0;
    ctrl.y       = 0.0;
    ctrl.yaw     = 0.0;
    ctrl.deltatime = 0.05;

    int max_steps = 500;
    bool reached  = false;
    for (int i = 0; i < max_steps; i++) {
        if (ctrl.reachedGoal()) {
            reached = true;
            break;
        }
        ctrl.computeCommand();
        ctrl.update();
    }
    EXPECT_TRUE(reached);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
