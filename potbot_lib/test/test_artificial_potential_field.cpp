#include <gtest/gtest.h>
#include <potbot_lib/artificial_potential_field.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::potential;

// コンストラクタ引数順序: (rows, cols, resolution, weight_attr, weight_rep, dtr, origin_x, origin_y)

// ============================================================
// ArtificialPotentialField 初期化テスト
// ============================================================

TEST(APFTest, DefaultConstructor)
{
    ArtificialPotentialField apf(5, 5, 1.0);
    auto* values = apf.getValues();
    EXPECT_EQ(values->size(), 25u);  // 5*5
}

TEST(APFTest, InitPotentialField)
{
    ArtificialPotentialField apf;
    apf.initPotentialField(7, 7, 0.5, 0.0, 0.0);
    auto* values = apf.getValues();
    EXPECT_EQ(values->size(), 49u);  // 7*7
}

// ============================================================
// setGoal() テスト
// ============================================================

TEST(APFTest, SetGoalByCoordinate)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setGoal(0.0, 0.0);
    Point goal = apf.getGoal();
    EXPECT_DOUBLE_EQ(goal.x, 0.0);
    EXPECT_DOUBLE_EQ(goal.y, 0.0);
}

TEST(APFTest, SetGoalMarksFieldGrid)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    // 5x5, res=1.0, origin=(0,0): x_shift=-2.5, x_min=-2.5, x_max=2.5
    apf.setGoal(0.0, 0.0);

    std::vector<size_t> result;
    apf.searchFieldInfo(result, GridInfo::IS_GOAL);
    EXPECT_GE(result.size(), 1u);
}

TEST(APFTest, SetGoalMarksAroundGoal)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setGoal(0.0, 0.0);

    std::vector<size_t> result;
    apf.searchFieldInfo(result, GridInfo::IS_AROUND_GOAL);
    EXPECT_GE(result.size(), 1u);
}

// ============================================================
// setRobot() テスト
// ============================================================

TEST(APFTest, SetRobotByCoordinate)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setRobot(-1.0, -1.0);
    Point robot = apf.getRobot();
    EXPECT_DOUBLE_EQ(robot.x, -1.0);
    EXPECT_DOUBLE_EQ(robot.y, -1.0);
}

TEST(APFTest, SetRobotMarksFieldGrid)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setRobot(0.0, 0.0);

    std::vector<size_t> result;
    apf.searchFieldInfo(result, GridInfo::IS_ROBOT);
    EXPECT_GE(result.size(), 1u);
}

// ============================================================
// setObstacle() / clearObstacles() テスト
// ============================================================

TEST(APFTest, SetObstacleByCoordinate)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setObstacle(1.0, 0.0);

    std::vector<Point> obs;
    apf.getObstacles(obs);
    ASSERT_EQ(obs.size(), 1u);
    EXPECT_DOUBLE_EQ(obs[0].x, 1.0);
    EXPECT_DOUBLE_EQ(obs[0].y, 0.0);
}

TEST(APFTest, SetMultipleObstacles)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setObstacle(1.0, 0.0);
    apf.setObstacle(-1.0, 0.0);
    apf.setObstacle(0.0, 1.0);

    std::vector<Point> obs;
    apf.getObstacles(obs);
    EXPECT_EQ(obs.size(), 3u);
}

TEST(APFTest, ClearObstacles)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setObstacle(1.0, 0.0);
    apf.setObstacle(-1.0, 0.0);
    apf.clearObstacles();

    std::vector<Point> obs;
    apf.getObstacles(obs);
    EXPECT_EQ(obs.size(), 0u);
}

TEST(APFTest, SetObstacleByEigenVector)
{
    ArtificialPotentialField apf(5, 5, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    Eigen::Vector2d vec(0.5, 0.5);
    apf.setObstacle(vec);

    std::vector<Point> obs;
    apf.getObstacles(obs);
    ASSERT_EQ(obs.size(), 1u);
    EXPECT_DOUBLE_EQ(obs[0].x, 0.5);
    EXPECT_DOUBLE_EQ(obs[0].y, 0.5);
}

// ============================================================
// createPotentialField() テスト
// ============================================================

TEST(APFTest, AttractionIncreaseWithDistanceToGoal)
{
    // ゴールから遠いほど吸引ポテンシャルが大きくなる
    // 11x11, res=1.0, wa=1.0, wr=0.0, dtr=100.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);

    apf.setGoal(0.0, 0.0);
    apf.createPotentialField();

    auto* values = apf.getValues();

    // ゴール付近（距離<1.5）と遠い場所（距離>3.0）の吸引値を比較
    double potential_near = std::numeric_limits<double>::infinity();
    double potential_far = 0.0;
    for (const auto& v : (*values))
    {
        double dist = std::hypot(v.x - 0.0, v.y - 0.0);
        if (dist < 1.5)
        {
            potential_near = std::min(potential_near, v.attraction);
        }
        if (dist > 3.0)
        {
            potential_far = std::max(potential_far, v.attraction);
        }
    }
    EXPECT_LT(potential_near, potential_far);
}

TEST(APFTest, RepulsionIncreaseNearObstacle)
{
    // 障害物に近いほど斥力ポテンシャルが大きくなる
    // 11x11, res=1.0, wa=0.0, wr=1.0, dtr=3.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, 3.0, 0.0, 0.0);

    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    auto* values = apf.getValues();

    // 障害物付近（0 < dist < 1.5）と遠い場所（dist > 3.5）の斥力値を比較
    double potential_close = 0.0, potential_far = 0.0;
    for (const auto& v : (*values))
    {
        double dist = std::hypot(v.x - 0.0, v.y - 0.0);
        if (dist > 0.0 && dist < 1.5)
        {
            potential_close = std::max(potential_close, v.repulsion);
        }
        if (dist > 3.5)
        {
            potential_far = std::max(potential_far, v.repulsion);
        }
    }
    EXPECT_GT(potential_close, 0.0);        // 障害物付近は0より大きい斥力
    EXPECT_DOUBLE_EQ(potential_far, 0.0);   // 閾値外は0
}

TEST(APFTest, TotalPotentialIsAttractionPlusRepulsion)
{
    ArtificialPotentialField apf(7, 7, 1.0, 0.1, 0.1, 0.3, 0.0, 0.0);
    apf.setGoal(1.0, 0.0);
    apf.setObstacle(-1.0, 0.0);
    apf.createPotentialField();

    auto* values = apf.getValues();
    for (const auto& v : (*values))
    {
        EXPECT_NEAR(v.potential, v.attraction + v.repulsion, 1e-9);
    }
}

TEST(APFTest, PotentialMinimumNearGoal)
{
    // ゴール付近の合計ポテンシャルが最小になる（障害物なし）
    // 11x11, res=1.0, wa=1.0, wr=0.0, dtr=100.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);

    apf.setGoal(0.0, 0.0);
    apf.createPotentialField();

    auto* values = apf.getValues();

    size_t min_idx = 0;
    double min_val = std::numeric_limits<double>::infinity();
    for (const auto& v : (*values))
    {
        if (v.potential < min_val)
        {
            min_val = v.potential;
            min_idx = v.index;
        }
    }

    // 最小ポテンシャルのグリッドはゴール付近にあるべき
    FieldGrid min_grid = apf.getValue(min_idx);
    double dist_to_goal = std::hypot(min_grid.x - 0.0, min_grid.y - 0.0);
    EXPECT_LT(dist_to_goal, 2.0);  // ゴールから2m以内（グリッド解像度=1.0を考慮）
}

TEST(APFTest, LocalMinimumDetection)
{
    // 障害物があるとローカルミニマムが検出される場合がある
    // 11x11, res=1.0, wa=1.0, wr=5.0, dtr=2.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 1.0, 5.0, 2.0, 0.0, 0.0);

    apf.setGoal(4.0, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    // createPotentialField()がクラッシュしないこと
    SUCCEED();
}

TEST(APFTest, GetAttractionFieldUpdatesValues)
{
    // 5x5, res=1.0, wa=1.0, wr=0.0, dtr=100.0, origin=(0,0)
    ArtificialPotentialField apf(5, 5, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    apf.setGoal(0.0, 0.0);
    apf.createPotentialField();

    Field attr_field;
    apf.getAttractionField(attr_field);

    // getAttractionFieldはvalues_.valueをattractionに設定する
    auto* values = apf.getValues();
    for (const auto& v : (*values))
    {
        EXPECT_NEAR(v.value, v.attraction, 1e-9);
    }
}

TEST(APFTest, GetRepulsionFieldUpdatesValues)
{
    // 5x5, res=1.0, wa=0.0, wr=1.0, dtr=2.0, origin=(0,0)
    ArtificialPotentialField apf(5, 5, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    Field rep_field;
    apf.getRepulsionField(rep_field);

    auto* values = apf.getValues();
    for (const auto& v : (*values))
    {
        EXPECT_NEAR(v.value, v.repulsion, 1e-9);
    }
}

// ============================================================
// setParams() テスト
// ============================================================

TEST(APFTest, SetParams)
{
    ArtificialPotentialField apf(5, 5, 1.0);
    apf.setParams(2.0, 3.0, 0.5);
    EXPECT_DOUBLE_EQ(apf.getDistanceThresholdRepulsionField(), 0.5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
