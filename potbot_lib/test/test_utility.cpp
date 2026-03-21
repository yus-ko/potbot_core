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

// ============================================================
// utility::get_vec テスト
// ============================================================

TEST(UtilityGetVecTest, GetVecPointEmptyInput)
{
    std::vector<Point> pts;
    auto result = utility::get_vec(pts);
    EXPECT_EQ(result.size(), 0u);
}

TEST(UtilityGetVecTest, GetVecPointSingleElement)
{
    std::vector<Point> pts = {Point(1.0, 2.0, 3.0)};
    auto result = utility::get_vec(pts);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(result[0].y(), 2.0, 1e-9);
    EXPECT_NEAR(result[0].z(), 3.0, 1e-9);
}

TEST(UtilityGetVecTest, GetVecPointSize)
{
    std::vector<Point> pts = {Point(0, 0, 0), Point(1, 1, 1), Point(2, 2, 2)};
    auto result = utility::get_vec(pts);
    EXPECT_EQ(result.size(), 3u);
}

TEST(UtilityGetVecTest, GetVecPoseEmptyInput)
{
    std::vector<Pose> poses;
    auto result = utility::get_vec(poses);
    EXPECT_EQ(result.size(), 0u);
}

TEST(UtilityGetVecTest, GetVecPoseSize)
{
    std::vector<Pose> poses = {Pose(1, 2, 3), Pose(4, 5, 6)};
    auto result = utility::get_vec(poses);
    EXPECT_EQ(result.size(), 2u);
}

TEST(UtilityGetVecTest, GetVecPoseTranslation)
{
    std::vector<Pose> poses = {Pose(1.0, 2.0, 3.0)};
    auto result = utility::get_vec(poses);
    ASSERT_EQ(result.size(), 1u);
    Eigen::Vector3d t = result[0].translation();
    EXPECT_NEAR(t.x(), 1.0, 1e-9);
    EXPECT_NEAR(t.y(), 2.0, 1e-9);
    EXPECT_NEAR(t.z(), 3.0, 1e-9);
}

// ============================================================
// utility::get_index テスト
// ============================================================

TEST(UtilityGetIndexTest, GetIndexFound)
{
    std::vector<Eigen::Vector2d> vec = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(3.0, 4.0)
    };
    int idx = utility::get_index(vec, Eigen::Vector2d(1.0, 2.0));
    EXPECT_EQ(idx, 1);
}

TEST(UtilityGetIndexTest, GetIndexNotFound)
{
    std::vector<Eigen::Vector2d> vec = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 2.0)
    };
    int idx = utility::get_index(vec, Eigen::Vector2d(9.0, 9.0));
    EXPECT_EQ(idx, -1);
}

TEST(UtilityGetIndexTest, GetIndexFirstElement)
{
    std::vector<Eigen::Vector2d> vec = {
        Eigen::Vector2d(5.0, 6.0),
        Eigen::Vector2d(7.0, 8.0)
    };
    int idx = utility::get_index(vec, Eigen::Vector2d(5.0, 6.0));
    EXPECT_EQ(idx, 0);
}

// ============================================================
// utility::vec_to_path テスト
// ============================================================

TEST(UtilityVecToPathTest, VecToPathEmptyInput)
{
    std::vector<Eigen::VectorXd> vectors;
    std::vector<Pose> path;
    utility::vec_to_path(vectors, path);
    EXPECT_EQ(path.size(), 0u);
}

TEST(UtilityVecToPathTest, VecToPathSingleElement)
{
    // 実装は vec(0), vec(1) のみ参照するため2次元で十分
    std::vector<Eigen::VectorXd> vectors;
    Eigen::VectorXd v(2);
    v << 1.5, 2.5;
    vectors.push_back(v);
    std::vector<Pose> path;
    utility::vec_to_path(vectors, path);
    ASSERT_EQ(path.size(), 1u);
    EXPECT_NEAR(path[0].position.x, 1.5, 1e-9);
    EXPECT_NEAR(path[0].position.y, 2.5, 1e-9);
}

// ============================================================
// utility::bezier 高レベル補間テスト
// ============================================================

TEST(UtilityBezierHighLevelTest, BezierHighLevelEmptyPath)
{
    std::vector<Pose> path_raw;
    std::vector<Pose> path_interpolated;
    bool result = utility::bezier(path_raw, path_interpolated);
    EXPECT_FALSE(result);
}

TEST(UtilityBezierHighLevelTest, BezierHighLevelSinglePoint)
{
    std::vector<Pose> path_raw = {Pose(1.0, 2.0)};
    std::vector<Pose> path_interpolated;
    bool result = utility::bezier(path_raw, path_interpolated);
    EXPECT_FALSE(result);
}

TEST(UtilityBezierHighLevelTest, BezierHighLevelMultiplePoints)
{
    std::vector<Pose> path_raw = {
        Pose(0.0, 0.0),
        Pose(1.0, 0.0),
        Pose(2.0, 1.0)
    };
    std::vector<Pose> path_interpolated;
    bool result = utility::bezier(path_raw, path_interpolated);
    EXPECT_TRUE(result);
    EXPECT_GT(path_interpolated.size(), 0u);
}

// ============================================================
// Pose::to_affine() テスト
// ============================================================

TEST(PoseAffineTest, ToAffineTranslation)
{
    // Pose(1,2,3,...) で to_affine().translation() が (1,2,3) を返すこと
    Pose p(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    Eigen::Affine3d aff = p.to_affine();
    EXPECT_NEAR(aff.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(aff.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(aff.translation().z(), 3.0, 1e-9);
}

TEST(PoseAffineTest, ToAffineIdentityRotation)
{
    // ゼロ回転のPoseで to_affine() の回転成分が単位行列に近いこと
    Pose p(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Eigen::Affine3d aff = p.to_affine();
    Eigen::Matrix3d R = aff.rotation();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_NEAR((R - I).norm(), 0.0, 1e-9);
}

// ============================================================
// Point::to_rotation() の直交行列テスト
// ============================================================

TEST(PointRotationTest, ToRotationOrthogonal)
{
    // Point(0, 0, π/4) で to_rotation() が直交行列（R*R.T ≈ I）であること
    Point p(0.0, 0.0, M_PI / 4.0);
    Eigen::Matrix3d R = p.to_rotation();
    Eigen::Matrix3d RRt = R * R.transpose();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_NEAR((RRt - I).norm(), 0.0, 1e-9);
}

TEST(PointRotationTest, ToRotationDeterminantOne)
{
    // to_rotation() の行列式が1であること（回転行列の条件）
    Point p(0.1, 0.2, 0.3);
    Eigen::Matrix3d R = p.to_rotation();
    EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

// ============================================================
// utility::is_containing() テスト
// ============================================================

TEST(UtilityIsContainingTest, IsContainingTrue)
{
    // is_containing(3, {1,2,3,4}) が true を返すこと
    std::vector<int> v = {1, 2, 3, 4};
    EXPECT_TRUE(utility::is_containing(3, v));
}

TEST(UtilityIsContainingTest, IsContainingFalse)
{
    // is_containing(5, {1,2,3,4}) が false を返すこと
    std::vector<int> v = {1, 2, 3, 4};
    EXPECT_FALSE(utility::is_containing(5, v));
}

TEST(UtilityIsContainingTest, IsContainingEqualsContains)
{
    // is_containing() が contains() と同じ結果を返すこと
    std::vector<int> v = {1, 2, 3, 4};
    EXPECT_EQ(utility::is_containing(3, v), utility::contains(3, v));
    EXPECT_EQ(utility::is_containing(5, v), utility::contains(5, v));
}

// ============================================================
// utility::get_vec(Pose) の Affine3d translation テスト
// ============================================================

TEST(UtilityGetVecAffineTest, GetVecPoseAffineTranslation)
{
    // 複数のPoseを get_vec() で変換して Affine3d の translation が正しいこと
    std::vector<Pose> poses = {
        Pose(1.0, 2.0, 3.0),
        Pose(4.0, 5.0, 6.0)
    };
    auto result = utility::get_vec(poses);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_NEAR(result[0].translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(result[0].translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(result[0].translation().z(), 3.0, 1e-9);
    EXPECT_NEAR(result[1].translation().x(), 4.0, 1e-9);
    EXPECT_NEAR(result[1].translation().y(), 5.0, 1e-9);
    EXPECT_NEAR(result[1].translation().z(), 6.0, 1e-9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
