#include <gtest/gtest.h>
#include <potbot_lib/interpolate.hpp>
#include <potbot_lib/utility.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace potbot_lib;
using namespace potbot_lib::interpolate;

// ============================================================
// linear() テスト
// ============================================================

TEST(InterpolateTest, LinearTwoPoints)
{
    std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {1.0, 1.0}};
    std::vector<Eigen::Vector2d> out;
    linear(in, 5, out);
    EXPECT_GE(out.size(), 2u);
}

TEST(InterpolateTest, LinearSinglePoint)
{
    std::vector<Eigen::Vector2d> in = {{3.0, 4.0}};
    std::vector<Eigen::Vector2d> out;
    linear(in, 5, out);
    ASSERT_EQ(out.size(), in.size());
    EXPECT_NEAR(out[0].x(), 3.0, 1e-9);
    EXPECT_NEAR(out[0].y(), 4.0, 1e-9);
}

TEST(InterpolateTest, LinearStartEndPreserved)
{
    std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {5.0, 5.0}};
    std::vector<Eigen::Vector2d> out;
    linear(in, 10, out);
    ASSERT_FALSE(out.empty());
    EXPECT_NEAR(out.front().x(), 0.0, 1e-9);
    EXPECT_NEAR(out.front().y(), 0.0, 1e-9);
    EXPECT_NEAR(out.back().x(), 5.0, 1e-9);
    EXPECT_NEAR(out.back().y(), 5.0, 1e-9);
}

// ============================================================
// spline() テスト
// ============================================================

TEST(InterpolateTest, SplineTwoPoints)
{
    std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {1.0, 1.0}};
    std::vector<Eigen::Vector2d> out;
    EXPECT_NO_THROW(spline(in, 5, out));
    EXPECT_GE(out.size(), 2u);
}

TEST(InterpolateTest, SplineMultiplePoints)
{
    std::vector<Eigen::Vector2d> in = {
        {0.0, 0.0}, {1.0, 2.0}, {3.0, 1.0}, {4.0, 3.0}
    };
    std::vector<Eigen::Vector2d> out;
    EXPECT_NO_THROW(spline(in, 20, out));
    // 有限値のみ追加されるため、20点以下になりうる
    EXPECT_GE(out.size(), 2u);
}

TEST(InterpolateTest, SplineSinglePoint)
{
    std::vector<Eigen::Vector2d> in = {{2.0, 3.0}};
    std::vector<Eigen::Vector2d> out;
    spline(in, 5, out);
    ASSERT_EQ(out.size(), in.size());
    EXPECT_NEAR(out[0].x(), 2.0, 1e-9);
    EXPECT_NEAR(out[0].y(), 3.0, 1e-9);
}

// ============================================================
// bezier(Vector2d) テスト
// ============================================================

TEST(InterpolateTest, BezierVector2dTwoPoints)
{
    std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {1.0, 1.0}};
    std::vector<Eigen::Vector2d> out;
    bezier(in, 5, out);
    EXPECT_FALSE(out.empty());
}

TEST(InterpolateTest, BezierVector2dSinglePoint)
{
    std::vector<Eigen::Vector2d> in = {{3.0, 7.0}};
    std::vector<Eigen::Vector2d> out;
    bezier(in, 5, out);
    ASSERT_EQ(out.size(), in.size());
    EXPECT_NEAR(out[0].x(), 3.0, 1e-9);
    EXPECT_NEAR(out[0].y(), 7.0, 1e-9);
}

TEST(InterpolateTest, BezierVector2dStartNearFirstPoint)
{
    std::vector<Eigen::Vector2d> in = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 2.0}};
    std::vector<Eigen::Vector2d> out;
    bezier(in, 20, out);
    ASSERT_FALSE(out.empty());
    EXPECT_NEAR(out.front().x(), 1.0, 1e-6);
    EXPECT_NEAR(out.front().y(), 2.0, 1e-6);
}

TEST(InterpolateTest, BezierVector2dEndNearLastPoint)
{
    // 実装は t=0~1 を 1/num_points 刻みで計算するため、浮動小数点誤差で
    // t=1.0 に到達しないことがある。最後の点は終点付近に収まることを確認する。
    std::vector<Eigen::Vector2d> in = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 2.0}};
    std::vector<Eigen::Vector2d> out;
    bezier(in, 100, out);
    ASSERT_FALSE(out.empty());
    EXPECT_NEAR(out.back().x(), 5.0, 0.1);
    EXPECT_NEAR(out.back().y(), 2.0, 0.1);
}

// ============================================================
// bezier(Pose) テスト
// ============================================================

TEST(InterpolateTest, BezierPoseTwoPoints)
{
    std::vector<Pose> in = {Pose(0.0, 0.0), Pose(1.0, 1.0)};
    std::vector<Pose> out;
    bezier(in, 5, out);
    EXPECT_FALSE(out.empty());
}

TEST(InterpolateTest, BezierPosePreservesXY)
{
    std::vector<Pose> pose_in = {Pose(0.0, 0.0), Pose(2.0, 2.0), Pose(4.0, 0.0)};
    std::vector<Pose> pose_out;
    bezier(pose_in, 10, pose_out);

    std::vector<Eigen::Vector2d> vec_in = {{0.0, 0.0}, {2.0, 2.0}, {4.0, 0.0}};
    std::vector<Eigen::Vector2d> vec_out;
    bezier(vec_in, 10, vec_out);

    ASSERT_EQ(pose_out.size(), vec_out.size());
    for (size_t i = 0; i < pose_out.size(); i++)
    {
        EXPECT_NEAR(pose_out[i].position.x, vec_out[i].x(), 1e-6);
        EXPECT_NEAR(pose_out[i].position.y, vec_out[i].y(), 1e-6);
    }
}

// ============================================================
// bezier(Point) テスト — 空実装
// ============================================================

TEST(InterpolateTest, BezierPointEmptyOrPassthrough)
{
    std::vector<Point> in = {Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)};
    std::vector<Point> out;
    EXPECT_NO_THROW(bezier(in, 5, out));
    // 空実装のため出力は空のまま
    EXPECT_TRUE(out.empty());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
