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
#include <potbot_lib/pid.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::controller;

// ============================================================
// applyLimit() パラメータ化テスト
// Navigation2スタイル: 様々なリミット・速度の組み合わせを網羅
// ============================================================

struct ApplyLimitParams
{
  double max_linear;    // 線速度上限
  double max_angular;   // 角速度上限
  double input_v;       // applyLimit前のv
  double input_omega;   // applyLimit前のomega
  double expected_v;    // applyLimit後の期待v
  double expected_omega; // applyLimit後の期待omega
};

class PIDApplyLimitTest : public ::testing::TestWithParam<ApplyLimitParams> {};

INSTANTIATE_TEST_SUITE_P(
  LimitCombinations,
  PIDApplyLimitTest,
  ::testing::Values(
    // v が上限を超える場合
    ApplyLimitParams{0.5, M_PI, 1.0, 0.0, 0.5, 0.0},
    // v が下限を超える場合（負値）
    ApplyLimitParams{0.5, M_PI, -1.0, 0.0, -0.5, 0.0},
    // omega が上限を超える場合
    ApplyLimitParams{1.0, 1.0, 0.0, 5.0, 0.0, 1.0},
    // omega が下限を超える場合（負値）
    ApplyLimitParams{1.0, 1.0, 0.0, -5.0, 0.0, -1.0},
    // v も omega もリミット内の場合：変化なし
    ApplyLimitParams{1.0, 2.0, 0.5, 1.0, 0.5, 1.0},
    // v も omega もゼロ：変化なし
    ApplyLimitParams{1.0, M_PI, 0.0, 0.0, 0.0, 0.0},
    // v と omega 両方が上限を超える場合
    ApplyLimitParams{0.3, 0.5, 2.0, 3.0, 0.3, 0.5}
  )
);

TEST_P(PIDApplyLimitTest, ApplyLimitClamps)
{
  auto p = GetParam();
  PID pid;
  pid.setLimit(p.max_linear, p.max_angular);
  pid.v = p.input_v;
  pid.omega = p.input_omega;
  pid.applyLimit();
  EXPECT_NEAR(pid.v, p.expected_v, 1e-9);
  EXPECT_NEAR(pid.omega, p.expected_omega, 1e-9);
}

// ============================================================
// reachedTarget() パラメータ化テスト
// ロボット位置と目標位置の組み合わせで到達判定を検証
// ============================================================

struct ReachedTargetParams
{
  double target_x;
  double target_y;
  double margin_dist;  // 距離マージン
  double margin_angle; // 角度マージン
  bool expected;       // 到達判定の期待値
};

class PIDReachedTargetTest : public ::testing::TestWithParam<ReachedTargetParams> {};

INSTANTIATE_TEST_SUITE_P(
  ReachedTargetCombinations,
  PIDReachedTargetTest,
  ::testing::Values(
    // ロボットと目標が同一点（原点）: 到達
    ReachedTargetParams{0.0, 0.0, 0.03, 0.1, true},
    // マージン外に目標がある: 未到達
    ReachedTargetParams{10.0, 10.0, 0.03, 0.1, false},
    // マージン内に目標がある: 到達
    ReachedTargetParams{0.02, 0.0, 0.03, 0.1, true},
    // マージン境界上（距離=margin_dist）: 到達とみなされるか実装依存
    // 距離がマージンを超える: 未到達
    ReachedTargetParams{1.0, 0.0, 0.03, 0.1, false},
    // 大きなマージン: 遠い目標でも到達
    ReachedTargetParams{0.5, 0.0, 1.0, M_PI, true}
  )
);

TEST_P(PIDReachedTargetTest, ReachedTargetCheck)
{
  auto p = GetParam();
  PID pid;
  // ロボットは原点（デフォルト）
  pid.setMargin(p.margin_angle, p.margin_dist);
  Pose target(p.target_x, p.target_y, 0.0, 0.0, 0.0, 0.0);
  pid.setTargetPoint(target);
  EXPECT_EQ(pid.reachedTarget(), p.expected);
}

// ============================================================
// ゲイン組み合わせによる収束特性パラメータ化テスト
// Navigation2スタイル: 高P/低P、I有無、D有無の組み合わせを検証
// ============================================================

struct GainConvergenceParams
{
  double gain_p;
  double gain_i;
  double gain_d;
  double target_x;    // X軸上の目標（正面に目標を置く）
  int max_steps;      // 最大ステップ数
  bool expect_converge; // 収束することを期待するか
};

class PIDGainConvergenceTest : public ::testing::TestWithParam<GainConvergenceParams> {};

INSTANTIATE_TEST_SUITE_P(
  GainCombinations,
  PIDGainConvergenceTest,
  ::testing::Values(
    // 高Pゲイン: 速く収束する
    GainConvergenceParams{5.0, 0.0, 0.0, 0.5, 2000, true},
    // 低Pゲイン: 収束に時間がかかるが収束する
    GainConvergenceParams{1.0, 0.0, 0.0, 0.5, 5000, true},
    // P+Iゲイン: 定常偏差なしで収束する
    GainConvergenceParams{3.0, 0.5, 0.0, 0.5, 5000, true},
    // P+Dゲイン（D微小）: オーバーシュートを抑えて収束する
    GainConvergenceParams{3.0, 0.0, 0.001, 0.5, 2000, true},
    // P+I+Dゲイン: 標準設定で収束する
    GainConvergenceParams{3.0, 0.5, 0.001, 0.5, 5000, true}
  )
);

TEST_P(PIDGainConvergenceTest, ConvergesWithGain)
{
  auto p = GetParam();
  PID pid;
  pid.setGain(p.gain_p, p.gain_i, p.gain_d);
  pid.setMargin(0.1, 0.05);
  pid.setLimit(1.0, M_PI);
  pid.deltatime = 0.05;

  Pose target(p.target_x, 0.0, 0.0, 0.0, 0.0, 0.0);
  pid.setTargetPoint(target);

  bool converged = false;
  for (int i = 0; i < p.max_steps; i++) {
    pid.calculateCommand();
    pid.update();
    if (pid.reachedTarget()) {
      converged = true;
      break;
    }
  }

  EXPECT_EQ(converged, p.expect_converge);
}

// ============================================================
// initPID() 後の状態リセットパラメータ化テスト
// 異なる目標設定後の initPID() でプロセスが PROCESS_STOP になることを検証
// ============================================================

struct InitPIDParams
{
  double target_x;
  double target_y;
  int steps_before_init; // initPID 前に実行するステップ数
};

class PIDInitResetTest : public ::testing::TestWithParam<InitPIDParams> {};

INSTANTIATE_TEST_SUITE_P(
  InitResetCombinations,
  PIDInitResetTest,
  ::testing::Values(
    InitPIDParams{1.0, 0.0, 1},
    InitPIDParams{0.0, 1.0, 5},
    InitPIDParams{2.0, 2.0, 10},
    InitPIDParams{-1.0, 0.0, 3}
  )
);

TEST_P(PIDInitResetTest, InitPIDResetsToStop)
{
  auto p = GetParam();
  PID pid;
  pid.setGain(3.0, 0.5, 0.001);
  pid.setLimit(1.0, M_PI);
  pid.deltatime = 0.05;

  Pose target(p.target_x, p.target_y, 0.0, 0.0, 0.0, 0.0);
  pid.setTargetPoint(target);

  // 指定ステップ数だけ実行してプロセスを変化させる
  for (int i = 0; i < p.steps_before_init; i++) {
    pid.calculateCommand();
    pid.update();
  }

  // initPID() 後はプロセスが PROCESS_STOP にリセットされること
  pid.initPID();
  EXPECT_EQ(pid.getCurrentProcess(), PROCESS_STOP);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
