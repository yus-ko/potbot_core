// Copyright 2024 potbot_core contributors
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
//
// リグレッションテスト: 既知エッジケースの再発防止
// Navigation2スタイル（nav2_costmap_2d/test/regression/）に準拠

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "potbot_lib/diff_drive_agent.hpp"
#include "potbot_lib/field.hpp"
#include "potbot_lib/artificial_potential_field.hpp"
#include "potbot_lib/apf_path_planner.hpp"
#include "potbot_lib/pid.hpp"
#include "potbot_lib/optimal_path_follower.hpp"

using namespace potbot_lib;
using namespace potbot_lib::potential;
using namespace potbot_lib::path_planner;
using namespace potbot_lib::controller;

// ==========================================
// REG-001: DiffDriveAgent - yaw角の数値安定性
// 背景: omega=10.0, dt=0.02 で1000回update()を連続実行しても
//       yaw・x・yがInf/NaNにならないことを確認する。
//       高角速度での連続回転でのオーバーフロー防止を目的とする。
// ==========================================
TEST(DiffDriveAgentRegression, YawAccumulationNoNanInf)
{
  DiffDriveAgent agent;
  agent.omega = 10.0;
  agent.deltatime = 0.02;

  for (int i = 0; i < 1000; i++) {
    agent.update();
  }

  EXPECT_FALSE(std::isinf(agent.yaw)) << "yawが無限大になった";
  EXPECT_FALSE(std::isnan(agent.yaw)) << "yawがNaNになった";
  EXPECT_FALSE(std::isinf(agent.x)) << "xが無限大になった";
  EXPECT_FALSE(std::isnan(agent.x)) << "xがNaNになった";
  EXPECT_FALSE(std::isinf(agent.y)) << "yが無限大になった";
  EXPECT_FALSE(std::isnan(agent.y)) << "yがNaNになった";
}

// ==========================================
// REG-002: DiffDriveAgent - 高速回転での例外なし
// 背景: omega=100.0という極端な値でも100回update()が
//       例外を送出せずに完了することを確認する。
//       テスト由来: UpdateYawAccumulationBeyondPi
// ==========================================
TEST(DiffDriveAgentRegression, HighSpeedRotationNoThrow)
{
  DiffDriveAgent agent;
  agent.omega = 100.0;
  agent.deltatime = 0.02;

  EXPECT_NO_THROW({
    for (int i = 0; i < 100; i++) {
      agent.update();
    }
  });
}

// ==========================================
// REG-003: DiffDriveAgent - 後退移動での数値安定性
// 背景: v=-1.0（後退）での1000回update()でInf/NaNが発生しないことを確認する。
//       負速度によるオーバーフロー防止を目的とする。
//       テスト由来: UpdateBackward
// ==========================================
TEST(DiffDriveAgentRegression, BackwardMotionNoNanInf)
{
  DiffDriveAgent agent;
  agent.v = -1.0;
  agent.deltatime = 0.02;

  for (int i = 0; i < 1000; i++) {
    agent.update();
  }

  EXPECT_FALSE(std::isinf(agent.x)) << "後退移動でxが無限大になった";
  EXPECT_FALSE(std::isnan(agent.x)) << "後退移動でxがNaNになった";
}

// ==========================================
// REG-004: Field - 範囲外座標でstd::out_of_rangeが発生すること
// 背景: getFieldIndex(100.0, 100.0) および checkIndex(100) が
//       std::out_of_rangeを投げることを確認する。
//       境界外アクセスの無効化によるメモリ安全性確保を目的とする。
//       テスト由来: GetFieldIndexOutOfRangeThrows, CheckIndexOutOfRangeThrows
// ==========================================
TEST(FieldRegression, OutOfRangeThrowsException)
{
  Field field(3, 3, 1.0, 0.0, 0.0);

  EXPECT_THROW(field.getFieldIndex(100.0, 100.0), std::out_of_range)
    << "範囲外座標でstd::out_of_rangeが投げられるべき";
  EXPECT_THROW(field.checkIndex(100), std::out_of_range)
    << "範囲外インデックスでstd::out_of_rangeが投げられるべき";
}

// ==========================================
// REG-005: Field - 極小グリッド(3x3)でのクラッシュなし
// 背景: rows=3, cols=3 という最小サイズのグリッドを構築しても
//       クラッシュしないことを確認する。
//       境界条件での初期化安全性を目的とする。
// ==========================================
TEST(FieldRegression, MinimalGridNoCrash)
{
  EXPECT_NO_THROW({
    Field field(3, 3, 1.0, 0.0, 0.0);
    auto * values = field.getValues();
    EXPECT_EQ(values->size(), 9u);
  });
}

// ==========================================
// REG-006: ApfPathPlanner - ロボット=ゴールでのクラッシュなし
// 背景: 起点と終点が同一座標(0,0)の場合、createPath()が
//       クラッシュせずに完了することを確認する。
//       ゼロ距離での除算・ループ無限化防止を目的とする。
//       テスト由来: RobotEqualsGoalNocrash
// ==========================================
TEST(ApfPathPlannerRegression, RobotAtGoalNoCrash)
{
  ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
  apf.setRobot(0.0, 0.0);
  apf.setGoal(0.0, 0.0);
  apf.createPotentialField();

  APFPathPlanner planner(&apf);
  EXPECT_NO_THROW({
    planner.createPath(0.0);
  });
}

// ==========================================
// REG-007: ApfPathPlanner - createPath()前のbezier()がクラッシュしない
// 背景: createPath()を呼ぶ前にbezier()を呼ぶと経路が空であり、
//       クラッシュせずにfalseを返すことを確認する。
//       空コンテナアクセスによる未定義動作防止を目的とする。
//       テスト由来: BezierOnEmptyPathReturnsFalse
// ==========================================
TEST(ApfPathPlannerRegression, BezierOnEmptyPathNoCrash)
{
  ArtificialPotentialField apf(11, 11, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);
  APFPathPlanner planner(&apf);

  EXPECT_NO_THROW({
    bool result = planner.bezier();
    EXPECT_FALSE(result) << "空経路に対してbezier()はfalseを返すべき";
  });
}

// ==========================================
// REG-008: ArtificialPotentialField - ロボット=ゴール位置でのクラッシュなし
// 背景: setRobot(0,0)とsetGoal(0,0)で同一座標を設定しても
//       createPotentialField()がクラッシュしないことを確認する。
//       ゼロ距離でのポテンシャル計算における除算エラー防止を目的とする。
//       テスト由来: RobotAndGoalSamePositionNoCrash
// ==========================================
TEST(APFRegression, RobotAndGoalSamePositionNoCrash)
{
  ArtificialPotentialField apf(7, 7, 1.0, 1.0, 1.0, 2.0, 0.0, 0.0);
  apf.setRobot(0.0, 0.0);
  apf.setGoal(0.0, 0.0);
  apf.setObstacle(1.0, 0.0);

  EXPECT_NO_THROW(apf.createPotentialField());
}

// ==========================================
// REG-009: ArtificialPotentialField - ゴール未設定でのcreateAttractionField()
// 背景: setGoal()を呼ばずにcreatePotentialField()を実行しても
//       クラッシュしないことを確認する。
//       未初期化状態でのゴール探索による例外・クラッシュ防止を目的とする。
// ==========================================
TEST(APFRegression, CreatePotentialFieldWithoutGoalNoCrash)
{
  ArtificialPotentialField apf(5, 5, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0);

  EXPECT_NO_THROW({
    apf.createPotentialField();
  });
}

// ==========================================
// REG-010: ArtificialPotentialField - 障害物なし状態での斥力はゼロ
// 背景: setObstacle()を呼ばずにcreateRepulsionField()相当の計算が実行されても
//       全グリッドの斥力がゼロであり数値的に正しいことを確認する。
//       障害物未設定時の誤った斥力計算防止を目的とする。
//       テスト由来: ClearObstaclesThenRepulsionIsZero
// ==========================================
TEST(APFRegression, NoObstacleRepulsionIsZero)
{
  ArtificialPotentialField apf(7, 7, 1.0, 0.0, 1.0, 3.0, 0.0, 0.0);
  apf.createPotentialField();

  auto * values = apf.getValues();
  for (const auto & v : (*values)) {
    EXPECT_DOUBLE_EQ(v.repulsion, 0.0) << "障害物未設定時の斥力はゼロであるべき";
  }
}

// ==========================================
// REG-011: OptimalPathFollower - 空経路でcalculateCommand()前はreachedTarget()=true
// 背景: setTargetPath()未呼び出し状態でreachedTarget()がtrueを返し、
//       クラッシュしないことを確認する。
//       空経路参照によるセグメンテーションフォールト防止を目的とする。
//       テスト由来: EmptyPathReachedTarget
// ==========================================
TEST(OptimalPathFollowerRegression, EmptyPathReachedTargetNoCrash)
{
  OptimalPathFollower follower;

  EXPECT_NO_THROW({
    bool result = follower.reachedTarget();
    EXPECT_TRUE(result) << "空経路時はreachedTarget()=trueであるべき";
  });
}

// ==========================================
// REG-012: OptimalPathFollower - setLimit(0,0,0,0)で速度はゼロ
// 背景: v_max=0, v_min=0に設定後、calculateCommand()を呼んでも
//       v=0, omega=0を維持することを確認する。
//       ゼロ制限下での速度オーバーシュート防止を目的とする。
//       テスト由来: ZeroLimitProducesZeroVelocity
// ==========================================
TEST(OptimalPathFollowerRegression, ZeroLimitProducesZeroVelocity)
{
  OptimalPathFollower follower;
  follower.setLimit(0.0, 0.0, 0.0, 0.0);
  follower.setOptimizationMethod("all_search");
  follower.setTimeIncrement(0.1);
  follower.setTimeEnd(0.5);
  follower.setLinearVelocityIncrement(0.1);
  follower.setAngularVelocityIncrement(0.5);

  std::vector<Pose> path;
  for (int i = 0; i <= 10; i++) {
    double t = static_cast<double>(i) / 10;
    path.push_back(Pose(t, 0.0, 0));
  }
  follower.setTargetPath(path);
  follower.calculateCommand();

  EXPECT_NEAR(follower.v, 0.0, 1e-9) << "v_max=0時はv=0であるべき";
  EXPECT_NEAR(follower.omega, 0.0, 1e-9) << "omega_max=0時はomega=0であるべき";
}

// ==========================================
// REG-013: PID - initPID()後のプロセス状態リセット
// 背景: calculateCommand()でプロセスが遷移した後にinitPID()を呼ぶと
//       プロセスがPROCESS_STOPにリセットされることを確認する。
//       状態リセット漏れによる制御ループ再起動時の誤動作防止を目的とする。
//       テスト由来: InitPIDResetsProcess
// ==========================================
TEST(PIDRegression, InitPIDResetsProcessToStop)
{
  PID pid;
  Pose target(1.0, 0.0, 0.0);
  pid.setTargetPoint(target);
  pid.calculateCommand();

  // calculateCommand()後はSTOP以外の状態になっているはず
  EXPECT_NE(pid.getCurrentProcess(), PROCESS_STOP);

  pid.initPID();

  EXPECT_EQ(pid.getCurrentProcess(), PROCESS_STOP)
    << "initPID()後はPROCESS_STOPにリセットされるべき";
  EXPECT_DOUBLE_EQ(pid.v, 0.0) << "initPID()後はv=0であるべき";
  EXPECT_DOUBLE_EQ(pid.omega, 0.0) << "initPID()後はomega=0であるべき";
}

// ==========================================
// REG-014: PID - setGain(0,0,0)でゼロ速度
// 背景: PIDゲインが全て0の場合、calculateCommand()を呼んでも
//       v=0, omega=0を維持することを確認する。
//       ゲインゼロ時の誤った速度出力防止を目的とする。
//       テスト由来: ZeroGainProducesZeroVelocity
// ==========================================
TEST(PIDRegression, ZeroGainProducesZeroVelocity)
{
  PID pid;
  pid.setGain(0.0, 0.0, 0.0);
  pid.setLimit(1.0, M_PI);
  pid.deltatime = 0.05;

  Pose target(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  pid.setTargetPoint(target);
  pid.calculateCommand();

  EXPECT_NEAR(pid.v, 0.0, 1e-9) << "ゲインゼロ時はv=0であるべき";
  EXPECT_NEAR(pid.omega, 0.0, 1e-9) << "ゲインゼロ時はomega=0であるべき";
}

// ==========================================
// REG-015: Field - infoFilter()で空フィールドを返しても例外なし
// 背景: IS_OBSTACLEが1つも設定されていない状態でinfoFilter()を呼んでも
//       クラッシュしないことを確認する。
//       空フィルタ結果の処理における例外・クラッシュ防止を目的とする。
// ==========================================
TEST(FieldRegression, InfoFilterOnEmptyFlagsNoCrash)
{
  Field field(3, 3, 1.0, 0.0, 0.0);

  EXPECT_NO_THROW({
    Field filtered;
    field.infoFilter(filtered, GridInfo::IS_OBSTACLE);
    auto * fv = filtered.getValues();
    EXPECT_EQ(fv->size(), 0u) << "IS_OBSTACLEが未設定の場合はフィルタ結果が空であるべき";
  });
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
