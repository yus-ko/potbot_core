#include <gtest/gtest.h>
#include <potbot_lib/diff_drive_agent.hpp>
#include <cmath>

using namespace potbot_lib;

// ============================================================
// DiffDriveAgent 基本テスト
// ============================================================

TEST(DiffDriveAgentTest, DefaultConstructor)
{
    DiffDriveAgent agent;
    EXPECT_DOUBLE_EQ(agent.x, 0.0);
    EXPECT_DOUBLE_EQ(agent.y, 0.0);
    EXPECT_DOUBLE_EQ(agent.yaw, 0.0);
    EXPECT_DOUBLE_EQ(agent.v, 0.0);
    EXPECT_DOUBLE_EQ(agent.omega, 0.0);
    EXPECT_DOUBLE_EQ(agent.deltatime, 0.02);
}

TEST(DiffDriveAgentTest, ParameterizedConstructor)
{
    DiffDriveAgent agent(1.0, 2.0, 0.5, 1.0, 0.1, 0.01);
    EXPECT_DOUBLE_EQ(agent.x, 1.0);
    EXPECT_DOUBLE_EQ(agent.y, 2.0);
    EXPECT_DOUBLE_EQ(agent.yaw, 0.5);
    EXPECT_DOUBLE_EQ(agent.v, 1.0);
    EXPECT_DOUBLE_EQ(agent.omega, 0.1);
    EXPECT_DOUBLE_EQ(agent.deltatime, 0.01);
}

// ============================================================
// update() 運動学テスト
// ============================================================

TEST(DiffDriveAgentTest, UpdateStraightForward)
{
    // yaw=0, v=1.0, omega=0 で直進
    DiffDriveAgent agent(0.0, 0.0, 0.0, 1.0, 0.0, 0.1);
    agent.update();
    // x += v*dt*cos(yaw) = 1.0*0.1*cos(0) = 0.1
    // y += v*dt*sin(yaw) = 1.0*0.1*sin(0) = 0
    // yaw += omega*dt = 0
    EXPECT_NEAR(agent.x, 0.1, 1e-9);
    EXPECT_NEAR(agent.y, 0.0, 1e-9);
    EXPECT_NEAR(agent.yaw, 0.0, 1e-9);
}

TEST(DiffDriveAgentTest, UpdateRotation)
{
    // yaw=0, v=0, omega=M_PI で旋回のみ
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, M_PI, 0.5);
    agent.update();
    // yaw += omega*dt = M_PI*0.5 = M_PI/2
    EXPECT_NEAR(agent.x, 0.0, 1e-9);
    EXPECT_NEAR(agent.y, 0.0, 1e-9);
    EXPECT_NEAR(agent.yaw, M_PI / 2.0, 1e-9);
}

TEST(DiffDriveAgentTest, UpdateFacing90Degrees)
{
    // yaw=M_PI/2 向きで v=1.0 直進 -> y方向に移動
    DiffDriveAgent agent(0.0, 0.0, M_PI / 2.0, 1.0, 0.0, 0.1);
    agent.update();
    // x += 1.0*0.1*cos(M_PI/2) ≈ 0
    // y += 1.0*0.1*sin(M_PI/2) = 0.1
    EXPECT_NEAR(agent.x, 0.0, 1e-9);
    EXPECT_NEAR(agent.y, 0.1, 1e-9);
    EXPECT_NEAR(agent.yaw, M_PI / 2.0, 1e-9);
}

TEST(DiffDriveAgentTest, UpdateOmegaBeforePosition)
{
    // update() はまず yaw を更新してから x,y を計算する
    // yaw=0, omega=M_PI/2, v=1.0, dt=0.5 の場合:
    // yaw_new = 0 + (M_PI/2)*0.5 = M_PI/4
    // x_new = 0 + 1.0*0.5*cos(M_PI/4) = 0.5*sqrt(2)/2
    // y_new = 0 + 1.0*0.5*sin(M_PI/4) = 0.5*sqrt(2)/2
    DiffDriveAgent agent(0.0, 0.0, 0.0, 1.0, M_PI / 2.0, 0.5);
    agent.update();
    double expected_yaw = M_PI / 4.0;
    EXPECT_NEAR(agent.yaw, expected_yaw, 1e-9);
    EXPECT_NEAR(agent.x, 0.5 * std::cos(expected_yaw), 1e-9);
    EXPECT_NEAR(agent.y, 0.5 * std::sin(expected_yaw), 1e-9);
}

TEST(DiffDriveAgentTest, UpdateMultipleSteps)
{
    // 複数ステップ更新
    DiffDriveAgent agent(0.0, 0.0, 0.0, 1.0, 0.0, 0.1);
    for (int i = 0; i < 10; i++)
    {
        agent.update();
    }
    // yaw=0で直進 -> x=1.0, y≈0
    EXPECT_NEAR(agent.x, 1.0, 1e-6);
    EXPECT_NEAR(agent.y, 0.0, 1e-6);
}

// ============================================================
// getDistance() テスト
// ============================================================

TEST(DiffDriveAgentTest, GetDistanceToPoint)
{
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Point p(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(agent.getDistance(p), 5.0);
}

TEST(DiffDriveAgentTest, GetDistanceToSamePosition)
{
    DiffDriveAgent agent(1.0, 1.0, 0.0, 0.0, 0.0, 0.02);
    Point p(1.0, 1.0, 0.0);
    EXPECT_DOUBLE_EQ(agent.getDistance(p), 0.0);
}

TEST(DiffDriveAgentTest, GetDistanceToPose)
{
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Pose p(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(agent.getDistance(p), 5.0);
}

TEST(DiffDriveAgentTest, GetDistanceNegativeCoords)
{
    DiffDriveAgent agent(1.0, 1.0, 0.0, 0.0, 0.0, 0.02);
    Point p(-2.0, -3.0, 0.0);
    // hypot(3, 4) = 5
    EXPECT_DOUBLE_EQ(agent.getDistance(p), 5.0);
}

// ============================================================
// getAngle() テスト
// ============================================================

TEST(DiffDriveAgentTest, GetAngleRight)
{
    // ロボットが原点、ターゲットが右(+x方向)
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Point p(1.0, 0.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), 0.0, 1e-9);
}

TEST(DiffDriveAgentTest, GetAngleUp)
{
    // ロボットが原点、ターゲットが上(+y方向)
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Point p(0.0, 1.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), M_PI / 2.0, 1e-9);
}

TEST(DiffDriveAgentTest, GetAngleLeft)
{
    // ロボットが原点、ターゲットが左(-x方向)
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Point p(-1.0, 0.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), M_PI, 1e-9);
}

TEST(DiffDriveAgentTest, GetAngleDown)
{
    // ロボットが原点、ターゲットが下(-y方向)
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Point p(0.0, -1.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), -M_PI / 2.0, 1e-9);
}

TEST(DiffDriveAgentTest, GetAngleToPose)
{
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Pose p(1.0, 1.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), M_PI / 4.0, 1e-9);
}

TEST(DiffDriveAgentTest, GetAngleOffset)
{
    // ロボットが(1,1)、ターゲットが(2,2)
    DiffDriveAgent agent(1.0, 1.0, 0.0, 0.0, 0.0, 0.02);
    Point p(2.0, 2.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), M_PI / 4.0, 1e-9);
}

// ============================================================
// エッジケーステスト
// ============================================================

TEST(DiffDriveAgentTest, UpdateBackward)
{
    // 後退移動テスト: v=-1.0, yaw=0, dt=0.1 で update() 後に x=-0.1 になること
    DiffDriveAgent agent(0.0, 0.0, 0.0, -1.0, 0.0, 0.1);
    agent.update();
    // x += v*dt*cos(yaw) = -1.0*0.1*cos(0) = -0.1
    EXPECT_NEAR(agent.x, -0.1, 1e-9);
    EXPECT_NEAR(agent.y, 0.0, 1e-9);
}

TEST(DiffDriveAgentTest, UpdateYawAccumulationBeyondPi)
{
    // yaw角が±πを超える継続更新: omega=1.0, dt=0.1 で20回update()後のyawが正しく累積されること（クラッシュしないこと）
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 1.0, 0.1);
    for (int i = 0; i < 20; i++)
    {
        agent.update();
    }
    // yaw = 1.0 * 0.1 * 20 = 2.0 [rad]（±πを超える可能性あり）
    // クラッシュせずに累積されること、累積値が概ね 2.0 であること
    EXPECT_NEAR(agent.yaw, 2.0, 1e-6);
}

TEST(DiffDriveAgentTest, UpdateDiagonal45Degrees)
{
    // 斜め45度方向への移動精度: yaw=π/4, v=1.0, dt=0.1 で update() 後に x≈y≈0.1/√2 になること
    DiffDriveAgent agent(0.0, 0.0, M_PI / 4.0, 1.0, 0.0, 0.1);
    agent.update();
    // x += v*dt*cos(M_PI/4) = 1.0*0.1*(√2/2) = 0.1/√2
    // y += v*dt*sin(M_PI/4) = 1.0*0.1*(√2/2) = 0.1/√2
    double expected = 0.1 / std::sqrt(2.0);
    EXPECT_NEAR(agent.x, expected, 1e-9);
    EXPECT_NEAR(agent.y, expected, 1e-9);
}

TEST(DiffDriveAgentTest, GetAnglePoseYDirection)
{
    // getAngle(Pose) のX-Y方向: ロボットが原点、target Pose が (0,1,0) の場合 getAngle=π/2 になること
    DiffDriveAgent agent(0.0, 0.0, 0.0, 0.0, 0.0, 0.02);
    Pose p(0.0, 1.0, 0.0);
    EXPECT_NEAR(agent.getAngle(p), M_PI / 2.0, 1e-9);
}

TEST(DiffDriveAgentTest, UpdateHighPrecisionMultipleSteps)
{
    // 複数ステップ後の位置精度: v=1.0, yaw=0, dt=0.01 で100回update後に x≈1.0 であること（誤差 1e-5以内）
    DiffDriveAgent agent(0.0, 0.0, 0.0, 1.0, 0.0, 0.01);
    for (int i = 0; i < 100; i++)
    {
        agent.update();
    }
    // x = 1.0 * 0.01 * 100 = 1.0
    EXPECT_NEAR(agent.x, 1.0, 1e-5);
    EXPECT_NEAR(agent.y, 0.0, 1e-5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
