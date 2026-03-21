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
#include <potbot_lib/interpolate.hpp>
#include <potbot_lib/utility.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace potbot_lib;
using namespace potbot_lib::interpolate;

// ============================================================
// bezier(Vector2d) の出力点数パラメータ化テスト
// — 異なる出力点数（num_points）で補間結果が空でないこと
// ============================================================

struct BezierNumPointsParams
{
  int num_points;    // 補間出力点数
};

class BezierVector2dNumPointsTest : public ::testing::TestWithParam<BezierNumPointsParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentNumPoints,
  BezierVector2dNumPointsTest,
  ::testing::Values(
    BezierNumPointsParams{3},
    BezierNumPointsParams{5},
    BezierNumPointsParams{10},
    BezierNumPointsParams{20},
    BezierNumPointsParams{50},
    BezierNumPointsParams{100},
    BezierNumPointsParams{200}
  )
);

// 補間結果が空でないこと
TEST_P(BezierVector2dNumPointsTest, OutputIsNotEmpty)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {
    {0.0, 0.0}, {1.0, 2.0}, {3.0, 1.0}, {4.0, 3.0}, {5.0, 0.0}
  };
  std::vector<Eigen::Vector2d> out;
  bezier(in, p.num_points, out);
  EXPECT_FALSE(out.empty());
}

// 開始点が制御点の先頭に近いこと
TEST_P(BezierVector2dNumPointsTest, StartPointNearFirstControlPoint)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 2.0}};
  std::vector<Eigen::Vector2d> out;
  bezier(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  // t=0 のとき B(0) = 制御点の先頭
  EXPECT_NEAR(out.front().x(), 1.0, 1e-6);
  EXPECT_NEAR(out.front().y(), 2.0, 1e-6);
}

// ============================================================
// bezier(Vector2d) の制御点数パラメータ化テスト
// — 異なる制御点数で補間結果が有効であること
// ============================================================

struct BezierControlPointsParams
{
  int num_control_points;  // 制御点数
};

class BezierVector2dControlPointsTest : public ::testing::TestWithParam<BezierControlPointsParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentControlPoints,
  BezierVector2dControlPointsTest,
  ::testing::Values(
    BezierControlPointsParams{2},
    BezierControlPointsParams{3},
    BezierControlPointsParams{5},
    BezierControlPointsParams{10},
    BezierControlPointsParams{20}
  )
);

// 制御点数が変わっても補間結果が空でないこと
TEST_P(BezierVector2dControlPointsTest, OutputIsNotEmpty)
{
  auto p = GetParam();
  // num_control_points 個の制御点を直線上に生成
  std::vector<Eigen::Vector2d> in;
  for (int i = 0; i < p.num_control_points; i++) {
    double t = static_cast<double>(i) / (p.num_control_points - 1);
    in.push_back({t * 5.0, t * 3.0});
  }
  std::vector<Eigen::Vector2d> out;
  bezier(in, 20, out);
  EXPECT_FALSE(out.empty());
}

// 制御点数が変わっても開始点が先頭制御点に近いこと
TEST_P(BezierVector2dControlPointsTest, StartPointNearFirstControlPoint)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in;
  for (int i = 0; i < p.num_control_points; i++) {
    double t = static_cast<double>(i) / (p.num_control_points - 1);
    in.push_back({t * 5.0, t * 3.0});
  }
  std::vector<Eigen::Vector2d> out;
  bezier(in, 20, out);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().x(), in.front().x(), 1e-6);
  EXPECT_NEAR(out.front().y(), in.front().y(), 1e-6);
}

// ============================================================
// linear() の補間点数パラメータ化テスト
// — 異なる num_points で補間結果の始終点が保たれること
// ============================================================

struct LinearNumPointsParams
{
  int num_points;  // 補間出力点数
};

class LinearInterpolateTest : public ::testing::TestWithParam<LinearNumPointsParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentNumPoints,
  LinearInterpolateTest,
  ::testing::Values(
    LinearNumPointsParams{3},
    LinearNumPointsParams{5},
    LinearNumPointsParams{10},
    LinearNumPointsParams{20},
    LinearNumPointsParams{50},
    LinearNumPointsParams{100}
  )
);

// 補間結果が空でないこと
TEST_P(LinearInterpolateTest, OutputIsNotEmpty)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {5.0, 5.0}};
  std::vector<Eigen::Vector2d> out;
  linear(in, p.num_points, out);
  EXPECT_GE(out.size(), 2u);
}

// 補間結果の始点が入力先頭点と一致すること
TEST_P(LinearInterpolateTest, StartPointPreserved)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {5.0, 5.0}};
  std::vector<Eigen::Vector2d> out;
  linear(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(out.front().y(), 0.0, 1e-9);
}

// 補間結果の終点が入力末尾点と一致すること
TEST_P(LinearInterpolateTest, EndPointPreserved)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {5.0, 5.0}};
  std::vector<Eigen::Vector2d> out;
  linear(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.back().x(), 5.0, 1e-9);
  EXPECT_NEAR(out.back().y(), 5.0, 1e-9);
}

// 全補間点が入力範囲 [0, 5] x [0, 5] の内側に収まること
TEST_P(LinearInterpolateTest, AllPointsWithinBounds)
{
  auto p = GetParam();
  std::vector<Eigen::Vector2d> in = {{0.0, 0.0}, {5.0, 5.0}};
  std::vector<Eigen::Vector2d> out;
  linear(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  for (const auto & pt : out) {
    EXPECT_GE(pt.x(), 0.0 - 1e-9);
    EXPECT_LE(pt.x(), 5.0 + 1e-9);
    EXPECT_GE(pt.y(), 0.0 - 1e-9);
    EXPECT_LE(pt.y(), 5.0 + 1e-9);
  }
}

// ============================================================
// bezier(Pose) の補間点数パラメータ化テスト
// — 異なる num_points で Pose 補間結果の位置成分が有効であること
// ============================================================

struct BezierPoseNumPointsParams
{
  int num_points;  // 補間出力点数
};

class BezierPoseTest : public ::testing::TestWithParam<BezierPoseNumPointsParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentNumPoints,
  BezierPoseTest,
  ::testing::Values(
    BezierPoseNumPointsParams{3},
    BezierPoseNumPointsParams{5},
    BezierPoseNumPointsParams{10},
    BezierPoseNumPointsParams{20},
    BezierPoseNumPointsParams{50}
  )
);

// Pose 補間結果が空でないこと
TEST_P(BezierPoseTest, OutputIsNotEmpty)
{
  auto p = GetParam();
  std::vector<Pose> in = {
    Pose(0.0, 0.0), Pose(2.0, 2.0), Pose(4.0, 0.0)
  };
  std::vector<Pose> out;
  bezier(in, p.num_points, out);
  EXPECT_FALSE(out.empty());
}

// Pose 補間の開始点が先頭制御点に近いこと
TEST_P(BezierPoseTest, StartPointNearFirstControlPoint)
{
  auto p = GetParam();
  std::vector<Pose> in = {
    Pose(0.0, 0.0), Pose(2.0, 2.0), Pose(4.0, 0.0)
  };
  std::vector<Pose> out;
  bezier(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().position.x, 0.0, 1e-6);
  EXPECT_NEAR(out.front().position.y, 0.0, 1e-6);
}

// Pose 補間の X 座標が制御点の X 範囲 [0, 4] 内に収まること
TEST_P(BezierPoseTest, XCoordinateWithinBounds)
{
  auto p = GetParam();
  std::vector<Pose> in = {
    Pose(0.0, 0.0), Pose(2.0, 2.0), Pose(4.0, 0.0)
  };
  std::vector<Pose> out;
  bezier(in, p.num_points, out);
  ASSERT_FALSE(out.empty());
  for (const auto & pose : out) {
    EXPECT_GE(pose.position.x, -0.1);
    EXPECT_LE(pose.position.x, 4.1);
  }
}

// Vector2d 版と Pose 版の bezier() が同一の座標値を生成すること
TEST_P(BezierPoseTest, ConsistentWithVector2dVersion)
{
  auto p = GetParam();
  std::vector<Pose> pose_in = {
    Pose(0.0, 0.0), Pose(2.0, 2.0), Pose(4.0, 0.0)
  };
  std::vector<Pose> pose_out;
  bezier(pose_in, p.num_points, pose_out);

  std::vector<Eigen::Vector2d> vec_in = {{0.0, 0.0}, {2.0, 2.0}, {4.0, 0.0}};
  std::vector<Eigen::Vector2d> vec_out;
  bezier(vec_in, p.num_points, vec_out);

  ASSERT_EQ(pose_out.size(), vec_out.size());
  for (size_t i = 0; i < pose_out.size(); i++) {
    EXPECT_NEAR(pose_out[i].position.x, vec_out[i].x(), 1e-6);
    EXPECT_NEAR(pose_out[i].position.y, vec_out[i].y(), 1e-6);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
