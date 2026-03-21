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

// ============================================================
// 複数障害物での斥力干渉テスト
// ============================================================

TEST(APFTest, MultipleObstaclesCreateRepulsionAtAllLocations)
{
    // 3つの障害物を設置してcreateObstacle()後、
    // 3箇所すべてに斥力フィールドが存在すること
    // 11x11, res=1.0, wa=0.0, wr=1.0, dtr=2.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0);

    apf.setObstacle(-3.0, 0.0);
    apf.setObstacle(0.0, 0.0);
    apf.setObstacle(3.0, 0.0);
    apf.createPotentialField();

    auto* values = apf.getValues();

    // 各障害物付近（距離 < 1.0）に斥力が存在するか確認
    bool repulsion_near_obs1 = false;
    bool repulsion_near_obs2 = false;
    bool repulsion_near_obs3 = false;

    for (const auto& v : (*values))
    {
        double dist1 = std::hypot(v.x - (-3.0), v.y - 0.0);
        double dist2 = std::hypot(v.x - 0.0,   v.y - 0.0);
        double dist3 = std::hypot(v.x - 3.0,   v.y - 0.0);

        if (dist1 > 0.0 && dist1 < 1.0 && v.repulsion > 0.0) repulsion_near_obs1 = true;
        if (dist2 > 0.0 && dist2 < 1.0 && v.repulsion > 0.0) repulsion_near_obs2 = true;
        if (dist3 > 0.0 && dist3 < 1.0 && v.repulsion > 0.0) repulsion_near_obs3 = true;
    }

    EXPECT_TRUE(repulsion_near_obs1) << "障害物1付近に斥力が存在すべき";
    EXPECT_TRUE(repulsion_near_obs2) << "障害物2付近に斥力が存在すべき";
    EXPECT_TRUE(repulsion_near_obs3) << "障害物3付近に斥力が存在すべき";
}

// ============================================================
// clearObstacles() 後の斥力ゼロ確認
// ============================================================

TEST(APFTest, ClearObstaclesThenRepulsionIsZero)
{
    // 障害物を設置してcreateObstacle()後、clearObstacles()して
    // createPotentialField()を再実行すると斥力がゼロになること
    // 11x11, res=1.0, wa=0.0, wr=1.0, dtr=3.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, 3.0, 0.0, 0.0);

    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    // 障害物をクリアして再計算
    apf.clearObstacles();
    apf.createPotentialField();

    auto* values = apf.getValues();
    for (const auto& v : (*values))
    {
        EXPECT_DOUBLE_EQ(v.repulsion, 0.0) << "clearObstacles()後は斥力がゼロであるべき";
    }
}

// ============================================================
// setParams() でweight変更後のポテンシャル変化テスト
// ============================================================

TEST(APFTest, SetParamsWeightAttractionAffectsPotential)
{
    // wa=1.0 と wa=2.0 で同じゴール位置の吸引ポテンシャルを比較し、
    // wa=2.0 の方が大きいこと
    // 11x11, res=1.0, origin=(0,0)
    ArtificialPotentialField apf1(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
    ArtificialPotentialField apf2(11, 11, 1.0, 2.0, 0.0, 100.0, 0.0, 0.0);

    apf1.setGoal(0.0, 0.0);
    apf1.createPotentialField();

    apf2.setGoal(0.0, 0.0);
    apf2.createPotentialField();

    // ゴールから離れた点（距離 > 2.0）での吸引ポテンシャルを比較
    auto* values1 = apf1.getValues();
    auto* values2 = apf2.getValues();
    ASSERT_EQ(values1->size(), values2->size());

    bool found_larger = false;
    for (size_t i = 0; i < values1->size(); ++i)
    {
        double dist = std::hypot((*values1)[i].x, (*values1)[i].y);
        if (dist > 2.0)
        {
            // wa=2.0 の方が吸引ポテンシャルが大きいはず
            EXPECT_GT((*values2)[i].attraction, (*values1)[i].attraction)
                << "wa=2.0の吸引ポテンシャルはwa=1.0より大きいべき (dist=" << dist << ")";
            found_larger = true;
        }
    }
    EXPECT_TRUE(found_larger) << "距離 > 2.0 のグリッドが少なくとも1つ存在すべき";
}

// ============================================================
// initPotentialField() での再初期化テスト
// ============================================================

TEST(APFTest, InitPotentialFieldResetsToZero)
{
    // 一度createPotentialField()後にinitPotentialField()を呼ぶと
    // ポテンシャルが0にリセットされること
    ArtificialPotentialField apf(7, 7, 1.0, 1.0, 1.0, 2.0, 0.0, 0.0);

    apf.setGoal(0.0, 0.0);
    apf.setObstacle(2.0, 0.0);
    apf.createPotentialField();

    // 再初期化
    apf.initPotentialField(7, 7, 1.0, 0.0, 0.0);

    auto* values = apf.getValues();
    for (const auto& v : (*values))
    {
        EXPECT_DOUBLE_EQ(v.potential,  0.0) << "initPotentialField()後はpotentialが0であるべき";
        EXPECT_DOUBLE_EQ(v.attraction, 0.0) << "initPotentialField()後はattractionが0であるべき";
        EXPECT_DOUBLE_EQ(v.repulsion,  0.0) << "initPotentialField()後はrepulsionが0であるべき";
    }
}

// ============================================================
// Robot == Goal 位置でのクラッシュなしテスト
// ============================================================

TEST(APFTest, RobotAndGoalSamePositionNoCrash)
{
    // setRobot(0,0)とsetGoal(0,0)で同じ位置にしても
    // createPotentialField()がクラッシュしないこと
    ArtificialPotentialField apf(7, 7, 1.0, 1.0, 1.0, 2.0, 0.0, 0.0);

    apf.setRobot(0.0, 0.0);
    apf.setGoal(0.0, 0.0);
    apf.setObstacle(1.0, 0.0);

    // クラッシュしないことを確認
    EXPECT_NO_THROW(apf.createPotentialField());
}

// ============================================================
// searchFieldInfo(IS_REPULSION_FIELD_INSIDE) テスト
// ============================================================

TEST(APFTest, SearchRepulsionFieldInsideDetected)
{
    // 障害物を設置してcreateObstacle()後、
    // IS_REPULSION_FIELD_INSIDEが1つ以上検出されること
    // 11x11, res=1.0, wa=0.0, wr=1.0, dtr=3.0, origin=(0,0)
    ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, 3.0, 0.0, 0.0);

    apf.setObstacle(0.0, 0.0);
    apf.createPotentialField();

    std::vector<size_t> result;
    apf.searchFieldInfo(result, GridInfo::IS_REPULSION_FIELD_INSIDE);

    EXPECT_GE(result.size(), 1u) << "IS_REPULSION_FIELD_INSIDEが少なくとも1つ検出されるべき";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
