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
#include <potbot_lib/diff_drive_agent.hpp>
#include <cmath>

using namespace potbot_lib;

// ============================================================
// update() パラメータ化テスト
// Navigation2スタイル: 複数の速度/タイムステップ組み合わせを網羅
// ============================================================

// update() の実装:
//   yaw += omega * dt
//   x   += v * cos(yaw) * dt  ← 新しいyawを使用
//   y   += v * sin(yaw) * dt  ← 新しいyawを使用

struct UpdateParams
{
  double v;
  double omega;
  double dt;
  double expected_x;
  double expected_y;
  double expected_yaw;
};

class DiffDriveUpdateTest : public ::testing::TestWithParam<UpdateParams> {};

INSTANTIATE_TEST_SUITE_P(
  VelocityCombinations,
  DiffDriveUpdateTest,
  ::testing::Values(
    // 直進（omega=0, yaw=0）: yaw_new=0, x=1.0*cos(0)*0.1=0.1, y=0
    UpdateParams{1.0, 0.0, 0.1, 0.1, 0.0, 0.0},
    // 旋回のみ（v=0）: yaw_new=M_PI*0.1, x=0, y=0
    UpdateParams{0.0, M_PI, 0.1, 0.0, 0.0, M_PI * 0.1},
    // 後退（v=-1.0, omega=0）: yaw_new=0, x=-0.1, y=0
    UpdateParams{-1.0, 0.0, 0.1, -0.1, 0.0, 0.0},
    // デフォルトdeltatime 0.02s で直進: yaw_new=0, x=0.02, y=0
    UpdateParams{1.0, 0.0, 0.02, 0.02, 0.0, 0.0},
    // 右旋回しながら前進（omega=-M_PI/2）:
    //   yaw_new = 0 + (-M_PI/2)*0.1 = -M_PI/20
    //   x = 1.0*cos(-M_PI/20)*0.1
    //   y = 1.0*sin(-M_PI/20)*0.1
    UpdateParams{
      1.0, -M_PI / 2.0, 0.1,
      1.0 * std::cos(-M_PI / 20.0) * 0.1,
      1.0 * std::sin(-M_PI / 20.0) * 0.1,
      -M_PI / 20.0}
  )
);

TEST_P(DiffDriveUpdateTest, UpdatePosition)
{
  auto p = GetParam();
  // 初期位置 (0,0,0)、速度 v/omega、タイムステップ dt でエージェント生成
  DiffDriveAgent agent(0.0, 0.0, 0.0, p.v, p.omega, p.dt);
  agent.update();
  EXPECT_NEAR(agent.x, p.expected_x, 1e-9);
  EXPECT_NEAR(agent.y, p.expected_y, 1e-9);
  EXPECT_NEAR(agent.yaw, p.expected_yaw, 1e-9);
}

// ============================================================
// getDistance() パラメータ化テスト
// ============================================================

struct DistanceParams
{
  double robot_x;
  double robot_y;
  double target_x;
  double target_y;
  double expected_distance;
};

class DiffDriveDistanceTest : public ::testing::TestWithParam<DistanceParams> {};

INSTANTIATE_TEST_SUITE_P(
  DistanceCombinations,
  DiffDriveDistanceTest,
  ::testing::Values(
    // 3-4-5 直角三角形
    DistanceParams{0.0, 0.0, 3.0, 4.0, 5.0},
    // 同一点（距離ゼロ）
    DistanceParams{0.0, 0.0, 0.0, 0.0, 0.0},
    // 負座標: hypot(2-(-1), 3-(-1)) = hypot(3, 4) = 5
    DistanceParams{-1.0, -1.0, 2.0, 3.0, 5.0},
    // X軸方向のみ
    DistanceParams{0.0, 0.0, 1.0, 0.0, 1.0},
    // Y軸方向のみ
    DistanceParams{0.0, 0.0, 0.0, 1.0, 1.0}
  )
);

TEST_P(DiffDriveDistanceTest, GetDistanceToPoint)
{
  auto p = GetParam();
  DiffDriveAgent agent(p.robot_x, p.robot_y, 0.0, 0.0, 0.0, 0.02);
  Point target(p.target_x, p.target_y, 0.0);
  EXPECT_NEAR(agent.getDistance(target), p.expected_distance, 1e-9);
}

// ============================================================
// getAngle() パラメータ化テスト
// ============================================================

struct AngleParams
{
  double robot_x;
  double robot_y;
  double target_x;
  double target_y;
  double expected_angle;
};

class DiffDriveAngleTest : public ::testing::TestWithParam<AngleParams> {};

INSTANTIATE_TEST_SUITE_P(
  AngleCombinations,
  DiffDriveAngleTest,
  ::testing::Values(
    // +X 方向: atan2(0,1) = 0
    AngleParams{0.0, 0.0, 1.0, 0.0, 0.0},
    // +Y 方向: atan2(1,0) = M_PI/2
    AngleParams{0.0, 0.0, 0.0, 1.0, M_PI / 2.0},
    // -X 方向: atan2(0,-1) = M_PI
    AngleParams{0.0, 0.0, -1.0, 0.0, M_PI},
    // -Y 方向: atan2(-1,0) = -M_PI/2
    AngleParams{0.0, 0.0, 0.0, -1.0, -M_PI / 2.0},
    // 45度方向: atan2(1,1) = M_PI/4
    AngleParams{0.0, 0.0, 1.0, 1.0, M_PI / 4.0},
    // オフセット付き: ロボット(1,1)、ターゲット(2,2) -> atan2(1,1) = M_PI/4
    AngleParams{1.0, 1.0, 2.0, 2.0, M_PI / 4.0}
  )
);

TEST_P(DiffDriveAngleTest, GetAngleToPoint)
{
  auto p = GetParam();
  DiffDriveAgent agent(p.robot_x, p.robot_y, 0.0, 0.0, 0.0, 0.02);
  Point target(p.target_x, p.target_y, 0.0);
  EXPECT_NEAR(agent.getAngle(target), p.expected_angle, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
