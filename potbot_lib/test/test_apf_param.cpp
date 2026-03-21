// Copyright 2024 potbot contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <potbot_lib/artificial_potential_field.hpp>
#include <cmath>
#include <limits>

using namespace potbot_lib;
using namespace potbot_lib::potential;

// コンストラクタ引数順序:
//   ArtificialPotentialField(rows, cols, resolution,
//                            weight_attr, weight_rep, dtr,
//                            origin_x, origin_y)

// ============================================================
// ゲインパラメータ化テスト
// ============================================================

struct APFGainParams
{
  double attr_gain;
  double rep_gain;
  double dtr;  // 斥力が効く距離閾値
};

class APFGainTest : public ::testing::TestWithParam<APFGainParams> {};

INSTANTIATE_TEST_SUITE_P(
  GainCombinations,
  APFGainTest,
  ::testing::Values(
    APFGainParams{1.0, 1.0, 0.5},   // 標準
    APFGainParams{2.0, 0.5, 0.3},   // 高引力・低斥力
    APFGainParams{0.5, 2.0, 1.0},   // 低引力・高斥力
    APFGainParams{0.1, 0.1, 0.1}    // 低ゲイン全般
  )
);

// createPotentialField()がクラッシュしないこと
TEST_P(APFGainTest, PotentialFieldIsGeneratedWithoutCrash)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, p.attr_gain, p.rep_gain, p.dtr, 0.0, 0.0);
  apf.setGoal(0.0, 0.0);
  apf.setObstacle(2.0, 0.0);
  EXPECT_NO_THROW(apf.createPotentialField());
}

// グリッドサイズが正しく初期化されること
TEST_P(APFGainTest, GridSizeIsCorrect)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, p.attr_gain, p.rep_gain, p.dtr, 0.0, 0.0);
  auto* values = apf.getValues();
  EXPECT_EQ(values->size(), 121u);  // 11*11
}

// potential = attraction + repulsion が全セルで成立すること
TEST_P(APFGainTest, TotalPotentialEqualsAttractionPlusRepulsion)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, p.attr_gain, p.rep_gain, p.dtr, 0.0, 0.0);
  apf.setGoal(0.0, 0.0);
  apf.setObstacle(2.0, 0.0);
  apf.createPotentialField();
  auto* values = apf.getValues();
  for (const auto& v : (*values))
  {
    EXPECT_NEAR(v.potential, v.attraction + v.repulsion, 1e-9);
  }
}

// ============================================================
// 引力ゲインスケール比較テスト
// ============================================================

struct APFAttrGainScaleParams
{
  double attr_gain_low;
  double attr_gain_high;
  double rep_gain;
  double dtr;
};

class APFAttrGainScaleTest : public ::testing::TestWithParam<APFAttrGainScaleParams> {};

INSTANTIATE_TEST_SUITE_P(
  AttrGainScales,
  APFAttrGainScaleTest,
  ::testing::Values(
    // 引力ゲイン1.0 vs 2.0
    APFAttrGainScaleParams{1.0, 2.0, 0.0, 100.0},
    // 引力ゲイン0.5 vs 1.5
    APFAttrGainScaleParams{0.5, 1.5, 0.0, 100.0},
    // 引力ゲイン0.1 vs 1.0
    APFAttrGainScaleParams{0.1, 1.0, 0.0, 100.0}
  )
);

// 高い引力ゲインの方がゴールから離れた場所でより大きな吸引ポテンシャルを持つこと
TEST_P(APFAttrGainScaleTest, HigherAttrGainProducesLargerAttraction)
{
  auto p = GetParam();
  ArtificialPotentialField apf_low(11, 11, 1.0, p.attr_gain_low, p.rep_gain, p.dtr, 0.0, 0.0);
  ArtificialPotentialField apf_high(11, 11, 1.0, p.attr_gain_high, p.rep_gain, p.dtr, 0.0, 0.0);

  apf_low.setGoal(0.0, 0.0);
  apf_low.createPotentialField();

  apf_high.setGoal(0.0, 0.0);
  apf_high.createPotentialField();

  auto* values_low  = apf_low.getValues();
  auto* values_high = apf_high.getValues();
  ASSERT_EQ(values_low->size(), values_high->size());

  bool found = false;
  for (size_t i = 0; i < values_low->size(); ++i)
  {
    double dist = std::hypot((*values_low)[i].x, (*values_low)[i].y);
    if (dist > 2.0)
    {
      // 高い引力ゲインの方が大きい吸引ポテンシャルを持つべき
      EXPECT_GT((*values_high)[i].attraction, (*values_low)[i].attraction)
        << "高attr_gainの方がattractionが大きいべき (dist=" << dist << ")";
      found = true;
    }
  }
  EXPECT_TRUE(found) << "距離 > 2.0 のグリッドが少なくとも1つ存在すべき";
}

// ============================================================
// 障害物距離（dtr）パラメータ化テスト
// ============================================================

struct APFDtrParams
{
  double dtr;              // 斥力が効く距離
  double obstacle_x;
  double obstacle_y;
  double far_dist;         // dtrを超える距離（斥力がゼロになるべき）
};

class APFDtrTest : public ::testing::TestWithParam<APFDtrParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentDtr,
  APFDtrTest,
  ::testing::Values(
    // dtr=1.0: 距離2.0以上は斥力ゼロ
    APFDtrParams{1.0, 0.0, 0.0, 2.0},
    // dtr=2.0: 距離3.0以上は斥力ゼロ
    APFDtrParams{2.0, 0.0, 0.0, 3.0},
    // dtr=3.0: 距離4.0以上は斥力ゼロ
    APFDtrParams{3.0, 0.0, 0.0, 4.0}
  )
);

// dtrより遠いグリッドの斥力がゼロであること
TEST_P(APFDtrTest, RepulsionIsZeroBeyondDtr)
{
  auto p = GetParam();
  // 11x11, res=1.0, wa=0.0 (引力なし), wr=1.0, origin=(0,0)
  ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, p.dtr, 0.0, 0.0);
  apf.setObstacle(p.obstacle_x, p.obstacle_y);
  apf.createPotentialField();

  auto* values = apf.getValues();
  for (const auto& v : (*values))
  {
    double dist = std::hypot(v.x - p.obstacle_x, v.y - p.obstacle_y);
    if (dist > p.far_dist)
    {
      EXPECT_DOUBLE_EQ(v.repulsion, 0.0)
        << "dtrを超えた距離(" << dist << ")では斥力がゼロであるべき";
    }
  }
}

// dtr内の障害物付近に斥力が存在すること
TEST_P(APFDtrTest, RepulsionExistsWithinDtr)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, 0.0, 1.0, p.dtr, 0.0, 0.0);
  apf.setObstacle(p.obstacle_x, p.obstacle_y);
  apf.createPotentialField();

  auto* values = apf.getValues();
  double max_repulsion = 0.0;
  for (const auto& v : (*values))
  {
    double dist = std::hypot(v.x - p.obstacle_x, v.y - p.obstacle_y);
    // 障害物から距離0より大きくdtr以内の範囲
    if (dist > 0.0 && dist < p.dtr)
    {
      max_repulsion = std::max(max_repulsion, v.repulsion);
    }
  }
  EXPECT_GT(max_repulsion, 0.0) << "dtr内に斥力が存在すべき";
}

// ============================================================
// ゴール位置パラメータ化テスト
// ============================================================

struct APFGoalParams
{
  double goal_x;
  double goal_y;
};

class APFGoalTest : public ::testing::TestWithParam<APFGoalParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentGoalPositions,
  APFGoalTest,
  ::testing::Values(
    APFGoalParams{0.0,  0.0},   // 中心
    APFGoalParams{2.0,  0.0},   // 右
    APFGoalParams{-2.0, 0.0},   // 左
    APFGoalParams{0.0,  2.0},   // 上
    APFGoalParams{0.0, -2.0}    // 下
  )
);

// 指定したゴール位置がgetGoal()で取得できること
TEST_P(APFGoalTest, SetGoalReturnsCorrectPosition)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
  apf.setGoal(p.goal_x, p.goal_y);
  Point goal = apf.getGoal();
  EXPECT_DOUBLE_EQ(goal.x, p.goal_x);
  EXPECT_DOUBLE_EQ(goal.y, p.goal_y);
}

// ゴール付近（距離<1.5）の合計ポテンシャルが、ゴールから遠い場所より小さいこと
TEST_P(APFGoalTest, PotentialMinimumNearGoal)
{
  auto p = GetParam();
  // 障害物なし: wa=1.0, wr=0.0, dtr=100.0（斥力なし）
  ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
  apf.setGoal(p.goal_x, p.goal_y);
  apf.createPotentialField();

  auto* values = apf.getValues();
  double potential_near = std::numeric_limits<double>::infinity();
  double potential_far  = -std::numeric_limits<double>::infinity();
  for (const auto& v : (*values))
  {
    double dist = std::hypot(v.x - p.goal_x, v.y - p.goal_y);
    if (dist < 1.5)
    {
      potential_near = std::min(potential_near, v.potential);
    }
    if (dist > 3.0)
    {
      potential_far = std::max(potential_far, v.potential);
    }
  }
  // ゴール付近の最小ポテンシャルは、遠方の最大ポテンシャルより小さいこと
  if (potential_near < std::numeric_limits<double>::infinity() &&
      potential_far > -std::numeric_limits<double>::infinity())
  {
    EXPECT_LT(potential_near, potential_far)
      << "ゴール付近のポテンシャルは遠方より小さいべき";
  }
}

// ============================================================
// 複数障害物パラメータ化テスト
// ============================================================

struct APFMultiObstacleParams
{
  double rep_gain;
  double dtr;
  // 障害物座標リスト（x, y）のペア
  std::vector<std::pair<double, double>> obstacles;
  size_t expected_obstacle_count;
};

class APFMultiObstacleTest : public ::testing::TestWithParam<APFMultiObstacleParams> {};

INSTANTIATE_TEST_SUITE_P(
  MultipleObstacles,
  APFMultiObstacleTest,
  ::testing::Values(
    APFMultiObstacleParams{1.0, 2.0, {{-3.0, 0.0}, {0.0, 0.0}, {3.0, 0.0}}, 3},
    APFMultiObstacleParams{0.5, 1.5, {{-2.0, -2.0}, {2.0, 2.0}}, 2},
    APFMultiObstacleParams{2.0, 3.0, {{-1.0, 0.0}}, 1}
  )
);

// getObstacles()が設定した障害物数を正しく返すこと
TEST_P(APFMultiObstacleTest, ObstacleCountMatches)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, 0.0, p.rep_gain, p.dtr, 0.0, 0.0);
  for (const auto& obs : p.obstacles)
  {
    apf.setObstacle(obs.first, obs.second);
  }
  std::vector<Point> obs_list;
  apf.getObstacles(obs_list);
  EXPECT_EQ(obs_list.size(), p.expected_obstacle_count);
}

// clearObstacles()後にgetObstacles()が空を返すこと
TEST_P(APFMultiObstacleTest, ClearObstaclesEmptiesList)
{
  auto p = GetParam();
  ArtificialPotentialField apf(11, 11, 1.0, 0.0, p.rep_gain, p.dtr, 0.0, 0.0);
  for (const auto& obs : p.obstacles)
  {
    apf.setObstacle(obs.first, obs.second);
  }
  apf.clearObstacles();
  std::vector<Point> obs_list;
  apf.getObstacles(obs_list);
  EXPECT_EQ(obs_list.size(), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
