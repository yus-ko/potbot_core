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
#include <potbot_lib/optimal_path_follower.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::controller;

// ============================================================
// ヘルパー関数: 始点(0,0)からgoal(goal_x, goal_y)へのPoseリストを生成する
// ============================================================
static std::vector<potbot_lib::Pose> make_path_param(double goal_x, double goal_y, int n)
{
  std::vector<potbot_lib::Pose> path;
  for (int i = 0; i <= n; i++) {
    double t = static_cast<double>(i) / n;
    path.push_back(potbot_lib::Pose(goal_x * t, goal_y * t, 0));
  }
  return path;
}

// ============================================================
// 経路長パラメータ化テスト — 異なる経路点数でのsetTargetPath/reachedTarget
// ============================================================

struct FollowerParams
{
  int path_length;       // 経路点数（区間数）
  double goal_x;         // ゴールX座標
  double goal_y;         // ゴールY座標
  bool expected_reached; // ゴールが近傍かどうか
  std::string description;
};

class OptimalPathFollowerPathLengthTest : public ::testing::TestWithParam<FollowerParams> {};

INSTANTIATE_TEST_SUITE_P(
  PathLengths,
  OptimalPathFollowerPathLengthTest,
  ::testing::Values(
    // 最小経路（2点）: ゴール遠方 → 未到達
    FollowerParams{2, 5.0, 0.0, false, "最小経路（2点）"},
    // 短い経路: ゴール遠方 → 未到達
    FollowerParams{5, 5.0, 0.0, false, "短い経路"},
    // 中程度の経路: ゴール遠方 → 未到達
    FollowerParams{10, 5.0, 0.0, false, "中程度の経路"},
    // 長い経路: ゴール遠方 → 未到達
    FollowerParams{20, 5.0, 0.0, false, "長い経路"},
    // 非常に長い経路: ゴール遠方 → 未到達
    FollowerParams{50, 5.0, 0.0, false, "非常に長い経路"},
    // 近傍ゴール（stop_margin_distance_=0.03m より小さい0.01m）→ 到達済み
    FollowerParams{10, 0.01, 0.0, true, "近傍ゴール（到達済み）"}
  )
);

// setTargetPath() 後に reachedTarget() が期待値通りであること
TEST_P(OptimalPathFollowerPathLengthTest, ReachedTargetMatchesExpectation)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  auto path = make_path_param(p.goal_x, p.goal_y, p.path_length);
  follower.setTargetPath(path);
  EXPECT_EQ(follower.reachedTarget(), p.expected_reached);
}

// setTargetPath() を呼んでも例外が発生しないこと
TEST_P(OptimalPathFollowerPathLengthTest, SetTargetPathNoThrow)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  auto path = make_path_param(p.goal_x, p.goal_y, p.path_length);
  EXPECT_NO_THROW(follower.setTargetPath(path));
}

// ============================================================
// 最適化手法パラメータ化テスト — all_search / gradient それぞれで
// calculateCommand() 後の速度が制限範囲内であること
// ============================================================

struct MethodParams
{
  std::string method;  // 最適化手法名
};

class OptimalPathFollowerMethodTest : public ::testing::TestWithParam<MethodParams> {};

INSTANTIATE_TEST_SUITE_P(
  OptimizationMethods,
  OptimalPathFollowerMethodTest,
  ::testing::Values(
    MethodParams{"all_search"},
    MethodParams{"gradient"}
  )
);

// calculateCommand() 後に v が [v_min, v_max] の範囲内であること
TEST_P(OptimalPathFollowerMethodTest, LinearVelocityWithinLimit)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  double v_min = -0.2, v_max = 0.2;
  double omega_min = -1.0, omega_max = 1.0;
  follower.setLimit(v_min, v_max, omega_min, omega_max);
  follower.setOptimizationMethod(p.method);
  follower.setTimeIncrement(0.1);
  follower.setTimeEnd(0.5);
  follower.setLinearVelocityIncrement(0.1);
  follower.setAngularVelocityIncrement(0.5);
  follower.setIterationMax(20);
  follower.setLearningRate(0.01);

  auto path = make_path_param(1.0, 0.0, 10);
  follower.setTargetPath(path);
  EXPECT_NO_THROW(follower.calculateCommand());

  EXPECT_GE(follower.v, v_min);
  EXPECT_LE(follower.v, v_max);
}

// calculateCommand() 後に omega が [omega_min, omega_max] の範囲内であること
TEST_P(OptimalPathFollowerMethodTest, AngularVelocityWithinLimit)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  double v_min = -0.2, v_max = 0.2;
  double omega_min = -1.0, omega_max = 1.0;
  follower.setLimit(v_min, v_max, omega_min, omega_max);
  follower.setOptimizationMethod(p.method);
  follower.setTimeIncrement(0.1);
  follower.setTimeEnd(0.5);
  follower.setLinearVelocityIncrement(0.1);
  follower.setAngularVelocityIncrement(0.5);
  follower.setIterationMax(20);
  follower.setLearningRate(0.01);

  auto path = make_path_param(1.0, 0.0, 10);
  follower.setTargetPath(path);
  EXPECT_NO_THROW(follower.calculateCommand());

  EXPECT_GE(follower.omega, omega_min);
  EXPECT_LE(follower.omega, omega_max);
}

// getBestCmd() が制限範囲内の速度を返すこと
TEST_P(OptimalPathFollowerMethodTest, BestCmdWithinLimit)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  double v_min = -0.2, v_max = 0.2;
  double omega_min = -1.0, omega_max = 1.0;
  follower.setLimit(v_min, v_max, omega_min, omega_max);
  follower.setOptimizationMethod(p.method);
  follower.setTimeIncrement(0.1);
  follower.setTimeEnd(0.5);
  follower.setLinearVelocityIncrement(0.1);
  follower.setAngularVelocityIncrement(0.5);
  follower.setIterationMax(20);
  follower.setLearningRate(0.01);

  auto path = make_path_param(1.0, 0.0, 10);
  follower.setTargetPath(path);
  follower.calculateCommand();

  double cmd_v = 0, cmd_omega = 0;
  EXPECT_NO_THROW(follower.getBestCmd(cmd_v, cmd_omega));
  EXPECT_GE(cmd_v, v_min);
  EXPECT_LE(cmd_v, v_max);
  EXPECT_GE(cmd_omega, omega_min);
  EXPECT_LE(cmd_omega, omega_max);
}

// ============================================================
// 目標方向パラメータ化テスト — 様々な方向の目標に対して
// calculateCommand() が何らかのコマンドを出力すること
// ============================================================

struct DirectionParams
{
  double goal_x;
  double goal_y;
  std::string description;
};

class OptimalPathFollowerDirectionTest : public ::testing::TestWithParam<DirectionParams> {};

INSTANTIATE_TEST_SUITE_P(
  GoalDirections,
  OptimalPathFollowerDirectionTest,
  ::testing::Values(
    // +X方向
    DirectionParams{1.0, 0.0, "+X方向"},
    // +Y方向
    DirectionParams{0.0, 1.0, "+Y方向"},
    // -X方向
    DirectionParams{-1.0, 0.0, "-X方向"},
    // -Y方向
    DirectionParams{0.0, -1.0, "-Y方向"},
    // 斜め（第一象限）
    DirectionParams{1.0, 1.0, "斜め（第一象限）"},
    // 斜め（第三象限）
    DirectionParams{-1.0, -1.0, "斜め（第三象限）"}
  )
);

// 様々な方向のゴールに対して v または omega が非ゼロであること
TEST_P(OptimalPathFollowerDirectionTest, ProducesCommandForDifferentDirections)
{
  auto p = GetParam();
  OptimalPathFollower follower;
  follower.setLimit(-0.2, 0.2, -1.0, 1.0);
  follower.setOptimizationMethod("all_search");
  follower.setTimeIncrement(0.1);
  follower.setTimeEnd(0.5);
  follower.setLinearVelocityIncrement(0.1);
  follower.setAngularVelocityIncrement(0.5);

  auto path = make_path_param(p.goal_x, p.goal_y, 10);
  follower.setTargetPath(path);
  EXPECT_NO_THROW(follower.calculateCommand());

  // v または omega が非ゼロであること（ゴールが遠方なので何かしら動くはず）
  bool has_cmd = (std::abs(follower.v) > 1e-9) || (std::abs(follower.omega) > 1e-9);
  EXPECT_TRUE(has_cmd);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
