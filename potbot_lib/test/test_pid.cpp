#include <gtest/gtest.h>
#include <potbot_lib/pid.hpp>
#include <cmath>

using namespace potbot_lib;
using namespace potbot_lib::controller;

// ============================================================
// PID 初期状態テスト
// ============================================================

TEST(PIDTest, InitialProcessIsStop)
{
    PID pid;
    EXPECT_EQ(pid.getCurrentProcess(), PROCESS_STOP);
}

TEST(PIDTest, InitialVelocitiesAreZero)
{
    PID pid;
    EXPECT_DOUBLE_EQ(pid.v, 0.0);
    EXPECT_DOUBLE_EQ(pid.omega, 0.0);
}

// ============================================================
// setTargetPoint / setGain / setMargin / setLimit テスト
// ============================================================

TEST(PIDTest, SetGain)
{
    PID pid;
    pid.setGain(2.0, 1.0, 0.5);
    // ゲインは protected なので直接確認は難しいが、
    // 挙動への影響でテストする（コンパイル確認）
    SUCCEED();
}

TEST(PIDTest, SetMargin)
{
    PID pid;
    pid.setMargin(0.05, 0.01);
    SUCCEED();
}

TEST(PIDTest, SetLimit)
{
    PID pid;
    pid.setLimit(0.5, 1.0);
    SUCCEED();
}

// ============================================================
// applyLimit() テスト
// ============================================================

TEST(PIDTest, ApplyLimitLinearVelocity)
{
    PID pid;
    pid.setLimit(0.5, M_PI);
    pid.v = 1.0;  // 上限を超えている
    pid.applyLimit();
    EXPECT_DOUBLE_EQ(pid.v, 0.5);
}

TEST(PIDTest, ApplyLimitLinearVelocityNegative)
{
    PID pid;
    pid.setLimit(0.5, M_PI);
    pid.v = -1.0;  // 下限を超えている
    pid.applyLimit();
    EXPECT_DOUBLE_EQ(pid.v, -0.5);
}

TEST(PIDTest, ApplyLimitAngularVelocity)
{
    PID pid;
    pid.setLimit(1.0, 1.0);
    pid.omega = 5.0;  // 上限を超えている
    pid.applyLimit();
    EXPECT_DOUBLE_EQ(pid.omega, 1.0);
}

TEST(PIDTest, ApplyLimitAngularVelocityNegative)
{
    PID pid;
    pid.setLimit(1.0, 1.0);
    pid.omega = -5.0;  // 下限を超えている
    pid.applyLimit();
    EXPECT_DOUBLE_EQ(pid.omega, -1.0);
}

TEST(PIDTest, ApplyLimitWithinBounds)
{
    PID pid;
    pid.setLimit(1.0, 2.0);
    pid.v = 0.5;
    pid.omega = 1.0;
    pid.applyLimit();
    EXPECT_DOUBLE_EQ(pid.v, 0.5);
    EXPECT_DOUBLE_EQ(pid.omega, 1.0);
}

// ============================================================
// reachedTarget() テスト
// ============================================================

TEST(PIDTest, ReachedTargetAtOrigin)
{
    PID pid;
    // ロボットと目標が同じ位置（デフォルト原点）
    Pose target(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    pid.setTargetPoint(target);
    // distance=0 <= margin=0.03 && angle差=0 <= margin=0.1
    EXPECT_TRUE(pid.reachedTarget());
}

TEST(PIDTest, NotReachedTargetFarAway)
{
    PID pid;
    Pose target(10.0, 10.0, 0.0, 0.0, 0.0, 0.0);
    pid.setTargetPoint(target);
    EXPECT_FALSE(pid.reachedTarget());
}

// ============================================================
// calculateCommand() / プロセス遷移テスト
// ============================================================

TEST(PIDTest, ProcessTransitionFromStop)
{
    PID pid;
    // 目標を遠い場所に設定
    Pose target(1.0, 0.0, 0.0);
    pid.setTargetPoint(target);
    pid.initPID();

    // calculateCommand() を呼ぶと PROCESS_STOP から PROCESS_ROTATE_DECLINATION に遷移するはず
    pid.calculateCommand();
    EXPECT_NE(pid.getCurrentProcess(), PROCESS_STOP);
}

TEST(PIDTest, InitPIDResetsProcess)
{
    PID pid;
    Pose target(1.0, 0.0, 0.0);
    pid.setTargetPoint(target);
    pid.calculateCommand();  // プロセスを変化させる

    pid.initPID();
    EXPECT_EQ(pid.getCurrentProcess(), PROCESS_STOP);
    EXPECT_DOUBLE_EQ(pid.v, 0.0);
    EXPECT_DOUBLE_EQ(pid.omega, 0.0);
}

TEST(PIDTest, CalculateCommandOutputsVelocity)
{
    PID pid;
    pid.setGain(2.0, 0.0, 0.0);

    // 正面に目標がある場合は直進コマンドが出るはず
    // ターゲットをX方向に設定
    Pose target(1.0, 0.0, 0.0);
    pid.setTargetPoint(target);
    pid.calculateCommand();
    pid.calculateCommand();  // 2回呼び、積分項が蓄積されるのを確認

    // 何らかの速度コマンドが出ていることを確認
    bool has_velocity = (std::abs(pid.v) > 0.0) || (std::abs(pid.omega) > 0.0);
    EXPECT_TRUE(has_velocity);
}

TEST(PIDTest, ConvergesToTarget)
{
    // ロボットを目標に向かって複数ステップ更新し、最終的に到達するか確認
    PID pid;
    pid.setGain(3.0, 0.0, 0.001);
    pid.setMargin(0.1, 0.05);
    pid.setLimit(1.0, M_PI);
    pid.deltatime = 0.05;

    Pose target(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    pid.setTargetPoint(target);

    // 最大1000ステップで収束を確認
    int max_steps = 1000;
    for (int i = 0; i < max_steps; i++)
    {
        pid.calculateCommand();
        pid.update();
        if (pid.reachedTarget()) break;
    }

    // 目標に到達していることを確認
    EXPECT_TRUE(pid.reachedTarget());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
