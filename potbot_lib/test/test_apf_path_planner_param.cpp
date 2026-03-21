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
#include <potbot_lib/apf_path_planner.hpp>
#include <potbot_lib/artificial_potential_field.hpp>
#include <cmath>
#include <string>

using namespace potbot_lib;
using namespace potbot_lib::path_planner;
using namespace potbot_lib::potential;

// ============================================================
// TEST_P パラメータ化テスト（Navigation2スタイル）
// ============================================================

// パラメータ構造体
// コンストラクタ引数順序: (rows, cols, resolution, weight_attr, weight_rep, dtr, origin_x, origin_y)
// 注意: createPath()内に norm() > 0.1 の距離チェックがあるため、
//       グリッド解像度を0.05m以下にする必要がある（対角距離 res*sqrt(2) < 0.1m）
struct PlannerParams
{
  int rows;
  int cols;
  double resolution;
  double start_x;
  double start_y;
  double goal_x;
  double goal_y;
  std::string description;  // テストの説明（デバッグ用）
};

class ApfPathPlannerParamTest : public ::testing::TestWithParam<PlannerParams> {};

INSTANTIATE_TEST_SUITE_P(
  GridConfigurations,
  ApfPathPlannerParamTest,
  ::testing::Values(
    // 41x41, res=0.05m の場合の座標範囲: x/y: [-1.025, 1.025]
    // 障害物なし（weight_rep=0.0）で各方向の経路生成を検証する
    PlannerParams{41, 41, 0.05, 0.0, 0.0, 0.5, 0.5, "高解像度グリッド正方向（斜め移動）"},
    PlannerParams{41, 41, 0.05, 0.0, 0.0, 0.0, 0.5, "Y軸方向移動"},
    PlannerParams{41, 41, 0.05, 0.0, 0.0, 0.5, 0.0, "X軸方向移動"},
    PlannerParams{41, 41, 0.05, -0.3, -0.3, 0.3, 0.3, "負座標から正座標（斜め移動）"},
    PlannerParams{41, 41, 0.05, 0.0, 0.0, -0.3, 0.3, "負方向ゴール（Y軸正、X軸負）"},
    // 51x51, res=0.04m の場合の座標範囲: x/y: [-1.02, 1.02]
    PlannerParams{51, 51, 0.04, -0.8, 0.0, 0.8, 0.0, "中グリッドX軸長距離移動"},
    // 81x81, res=0.025m の場合の座標範囲: x/y: [-1.0125, 1.0125]
    PlannerParams{81, 81, 0.025, 0.0, -0.9, 0.0, 0.9, "大グリッドY軸長距離移動"},
    // 短距離移動（隣接グリッド程度）
    PlannerParams{41, 41, 0.05, 0.0, 0.0, 0.1, 0.0, "短距離X軸移動"},
    // 負座標領域内での移動
    PlannerParams{41, 41, 0.05, -0.5, -0.5, -0.1, -0.1, "負座標領域内移動"}
  ),
  // テストスイート名にdescriptionを使用するカスタム名前付け
  [](const ::testing::TestParamInfo<PlannerParams> & info) {
    // テスト名に使用可能な文字のみ残す（スペースとマルチバイト文字をアンダースコアへ変換）
    std::string name = std::to_string(info.index) + "_case";
    return name;
  }
);

// ============================================================
// テスト1: createPath()が成功（trueを返す）するか
// ============================================================
TEST_P(ApfPathPlannerParamTest, CreatePathSucceeds)
{
  const PlannerParams & p = GetParam();

  // APFを生成: 障害物なし（weight_rep=0.0）、引力のみ
  ArtificialPotentialField apf(
    p.rows, p.cols, p.resolution,
    1.0,   // weight_attr
    0.0,   // weight_rep
    10.0,  // dtr
    0.0,   // origin_x
    0.0    // origin_y
  );
  apf.setRobot(p.start_x, p.start_y);
  apf.setGoal(p.goal_x, p.goal_y);
  apf.createPotentialField();

  APFPathPlanner planner(&apf);
  planner.setParams(3.0, 1, 1.0, 0.0);
  bool result = planner.createPath(0.0);

  EXPECT_TRUE(result) << "createPath() failed for: " << p.description;
}

// ============================================================
// テスト2: 生成された経路がstart位置から始まるか
// ============================================================
TEST_P(ApfPathPlannerParamTest, PathStartsNearRobot)
{
  const PlannerParams & p = GetParam();

  ArtificialPotentialField apf(
    p.rows, p.cols, p.resolution,
    1.0, 0.0, 10.0, 0.0, 0.0
  );
  apf.setRobot(p.start_x, p.start_y);
  apf.setGoal(p.goal_x, p.goal_y);
  apf.createPotentialField();

  APFPathPlanner planner(&apf);
  planner.setParams(3.0, 1, 1.0, 0.0);
  planner.createPath(0.0);

  std::vector<Pose> path;
  planner.getPath(path);
  ASSERT_GT(path.size(), 0u) << "Path is empty for: " << p.description;

  // 経路の最初の点はロボット位置付近（解像度の数グリッド以内）
  Point robot = apf.getRobot();
  double dist_start = std::hypot(
    path[0].position.x - robot.x,
    path[0].position.y - robot.y);
  // 許容誤差: グリッド解像度の2倍（1グリッド分以内）
  double tolerance = p.resolution * 2.0;
  EXPECT_LT(dist_start, tolerance)
    << "Path start is too far from robot for: " << p.description
    << " (dist=" << dist_start << ", tolerance=" << tolerance << ")";
}

// ============================================================
// テスト3: 生成された経路がgoal位置で終わるか
// ============================================================
TEST_P(ApfPathPlannerParamTest, PathEndsNearGoal)
{
  const PlannerParams & p = GetParam();

  ArtificialPotentialField apf(
    p.rows, p.cols, p.resolution,
    1.0, 0.0, 10.0, 0.0, 0.0
  );
  apf.setRobot(p.start_x, p.start_y);
  apf.setGoal(p.goal_x, p.goal_y);
  apf.createPotentialField();

  APFPathPlanner planner(&apf);
  planner.setParams(3.0, 1, 1.0, 0.0);
  planner.createPath(0.0);

  std::vector<Pose> path;
  planner.getPath(path);
  ASSERT_GT(path.size(), 0u) << "Path is empty for: " << p.description;

  // 経路の最後の点はゴール付近（解像度の6グリッド以内）
  Point goal = apf.getGoal();
  double dist_end = std::hypot(
    path.back().position.x - goal.x,
    path.back().position.y - goal.y);
  // 許容誤差: 0.3m（6グリッド相当 @ res=0.05m）
  EXPECT_LT(dist_end, 0.3)
    << "Path end is too far from goal for: " << p.description
    << " (dist=" << dist_end << ")";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
