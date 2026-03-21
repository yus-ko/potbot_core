#include <gtest/gtest.h>
#include <potbot_lib/utility.hpp>
#include <cmath>

using namespace potbot_lib;

// ============================================================
// Point 構造体テスト
// ============================================================

TEST(PointTest, DefaultConstructor)
{
    Point p;
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
    EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST(PointTest, ParameterizedConstructor)
{
    Point p(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.y, 2.0);
    EXPECT_DOUBLE_EQ(p.z, 3.0);
}

TEST(PointTest, EigenVector3dConstructor)
{
    Eigen::Vector3d v(4.0, 5.0, 6.0);
    Point p(v);
    EXPECT_DOUBLE_EQ(p.x, 4.0);
    EXPECT_DOUBLE_EQ(p.y, 5.0);
    EXPECT_DOUBLE_EQ(p.z, 6.0);
}

TEST(PointTest, Addition)
{
    Point a(1.0, 2.0, 3.0);
    Point b(4.0, 5.0, 6.0);
    Point c = a + b;
    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);
}

TEST(PointTest, Subtraction)
{
    Point a(5.0, 7.0, 9.0);
    Point b(1.0, 2.0, 3.0);
    Point c = a - b;
    EXPECT_DOUBLE_EQ(c.x, 4.0);
    EXPECT_DOUBLE_EQ(c.y, 5.0);
    EXPECT_DOUBLE_EQ(c.z, 6.0);
}

TEST(PointTest, ScalarMultiplication)
{
    Point a(1.0, 2.0, 3.0);
    Point c = a * 2.0;
    EXPECT_DOUBLE_EQ(c.x, 2.0);
    EXPECT_DOUBLE_EQ(c.y, 4.0);
    EXPECT_DOUBLE_EQ(c.z, 6.0);
}

TEST(PointTest, ScalarDivision)
{
    Point a(4.0, 6.0, 8.0);
    Point c = a / 2.0;
    EXPECT_DOUBLE_EQ(c.x, 2.0);
    EXPECT_DOUBLE_EQ(c.y, 3.0);
    EXPECT_DOUBLE_EQ(c.z, 4.0);
}

TEST(PointTest, Norm)
{
    Point p(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(p.norm(), 5.0);
}

TEST(PointTest, NormZeroVector)
{
    Point p(0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(p.norm(), 0.0);
}

TEST(PointTest, Norm3D)
{
    // 3,4,0 → 5 の3D版: sqrt(1+4+9)=sqrt(14)
    Point p(1.0, 2.0, 3.0);
    EXPECT_NEAR(p.norm(), std::sqrt(14.0), 1e-9);
}

TEST(PointTest, EqualityOperator)
{
    Point a(1.0, 2.0, 3.0);
    Point b(1.0, 2.0, 3.0);
    Point c(1.0, 2.0, 4.0);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TEST(PointTest, InequalityOperator)
{
    Point a(1.0, 2.0, 3.0);
    Point b(1.0, 2.0, 4.0);
    EXPECT_TRUE(a != b);
}

TEST(PointTest, ToTranslation)
{
    Point p(1.0, 2.0, 3.0);
    Eigen::Vector3d v = p.to_translation();
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

// ============================================================
// Pose 構造体テスト
// ============================================================

TEST(PoseTest, DefaultConstructor)
{
    Pose p;
    EXPECT_DOUBLE_EQ(p.position.x, 0.0);
    EXPECT_DOUBLE_EQ(p.position.y, 0.0);
    EXPECT_DOUBLE_EQ(p.position.z, 0.0);
    EXPECT_DOUBLE_EQ(p.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(p.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(p.rotation.z, 0.0);
}

TEST(PoseTest, ParameterizedConstructor)
{
    Pose p(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    EXPECT_DOUBLE_EQ(p.position.x, 1.0);
    EXPECT_DOUBLE_EQ(p.position.y, 2.0);
    EXPECT_DOUBLE_EQ(p.position.z, 3.0);
    EXPECT_DOUBLE_EQ(p.rotation.x, 0.1);
    EXPECT_DOUBLE_EQ(p.rotation.y, 0.2);
    EXPECT_DOUBLE_EQ(p.rotation.z, 0.3);
}

TEST(PoseTest, Addition)
{
    Pose a(1.0, 2.0, 0.0, 0.0, 0.0, 0.1);
    Pose b(3.0, 4.0, 0.0, 0.0, 0.0, 0.2);
    Pose c = a + b;
    EXPECT_DOUBLE_EQ(c.position.x, 4.0);
    EXPECT_DOUBLE_EQ(c.position.y, 6.0);
    EXPECT_NEAR(c.rotation.z, 0.3, 1e-9);
}

TEST(PoseTest, Subtraction)
{
    Pose a(5.0, 6.0, 0.0, 0.0, 0.0, 0.5);
    Pose b(1.0, 2.0, 0.0, 0.0, 0.0, 0.1);
    Pose c = a - b;
    EXPECT_DOUBLE_EQ(c.position.x, 4.0);
    EXPECT_DOUBLE_EQ(c.position.y, 4.0);
    EXPECT_NEAR(c.rotation.z, 0.4, 1e-9);
}

TEST(PoseTest, ScalarMultiplication)
{
    Pose a(2.0, 3.0, 0.0, 0.0, 0.0, 1.0);
    Pose c = a * 2.0;
    EXPECT_DOUBLE_EQ(c.position.x, 4.0);
    EXPECT_DOUBLE_EQ(c.position.y, 6.0);
    EXPECT_DOUBLE_EQ(c.rotation.z, 2.0);
}

TEST(PoseTest, EqualityOperator)
{
    Pose a(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    Pose b(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    Pose c(1.0, 2.0, 3.0, 0.1, 0.2, 0.4);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

// ============================================================
// utility 関数テスト
// ============================================================

TEST(UtilityTest, CombinationBasic)
{
    // nCr: 4C2 = 6
    EXPECT_DOUBLE_EQ(utility::combination(4, 2), 6.0);
}

TEST(UtilityTest, CombinationZero)
{
    // nC0 = 1
    EXPECT_DOUBLE_EQ(utility::combination(5, 0), 1.0);
}

TEST(UtilityTest, CombinationN)
{
    // nCn = 1
    EXPECT_DOUBLE_EQ(utility::combination(5, 5), 1.0);
}

TEST(UtilityTest, GetRotateMatrix)
{
    // 90度回転行列: [[0,-1],[1,0]]
    double th = M_PI / 2.0;
    Eigen::Matrix2d R = utility::get_rotate_matrix(th);
    EXPECT_NEAR(R(0, 0), 0.0, 1e-9);
    EXPECT_NEAR(R(0, 1), -1.0, 1e-9);
    EXPECT_NEAR(R(1, 0), 1.0, 1e-9);
    EXPECT_NEAR(R(1, 1), 0.0, 1e-9);
}

TEST(UtilityTest, GetRotateMatrixZero)
{
    // 0度回転行列: 単位行列
    Eigen::Matrix2d R = utility::get_rotate_matrix(0.0);
    EXPECT_NEAR(R(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(R(0, 1), 0.0, 1e-9);
    EXPECT_NEAR(R(1, 0), 0.0, 1e-9);
    EXPECT_NEAR(R(1, 1), 1.0, 1e-9);
}

TEST(UtilityTest, ContainsVector)
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    EXPECT_TRUE(utility::contains(3, v));
    EXPECT_FALSE(utility::contains(6, v));
}

TEST(UtilityTest, ContainsMap)
{
    std::map<std::string, int> m = {{"a", 1}, {"b", 2}};
    EXPECT_TRUE(utility::contains(std::string("a"), m));
    EXPECT_FALSE(utility::contains(std::string("c"), m));
}

TEST(UtilityTest, FindClosestVector)
{
    std::vector<Eigen::Vector2d> vectors = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(2.0, 0.0)
    };
    Eigen::Vector2d target(0.9, 0.0);
    Eigen::Vector2d closest;
    utility::find_closest_vector(vectors, target, closest);
    EXPECT_NEAR(closest.x(), 1.0, 1e-9);
    EXPECT_NEAR(closest.y(), 0.0, 1e-9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
