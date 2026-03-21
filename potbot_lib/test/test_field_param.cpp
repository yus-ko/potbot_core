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
#include <potbot_lib/field.hpp>
#include <cmath>
#include <stdexcept>

using namespace potbot_lib;
using namespace potbot_lib::potential;

// ============================================================
// グリッドサイズパラメータ化テスト
// ============================================================

struct FieldSizeParams
{
  size_t rows;
  size_t cols;
  double resolution;
  size_t expected_total_cells;  // rows * cols
};

class FieldSizeTest : public ::testing::TestWithParam<FieldSizeParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentSizes,
  FieldSizeTest,
  ::testing::Values(
    FieldSizeParams{11, 11, 1.0, 121},
    FieldSizeParams{21, 21, 0.5, 441},
    FieldSizeParams{41, 41, 0.05, 1681},
    FieldSizeParams{5, 10, 1.0, 50},    // 非正方形
    FieldSizeParams{101, 101, 0.1, 10201}
  )
);

// セル数がrows*colsに一致すること
TEST_P(FieldSizeTest, CellCountMatchesDimensions)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  auto* values = field.getValues();
  EXPECT_EQ(values->size(), p.expected_total_cells);
}

// ヘッダーのrows/colsが指定値と一致すること
TEST_P(FieldSizeTest, HeaderRowsColsMatch)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  FieldHeader h = field.getHeader();
  EXPECT_EQ(h.rows, p.rows);
  EXPECT_EQ(h.cols, p.cols);
  EXPECT_DOUBLE_EQ(h.resolution, p.resolution);
}

// width/heightがresolution*cols/rowsと一致すること
TEST_P(FieldSizeTest, HeaderWidthHeightMatchResolutionTimesSize)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  FieldHeader h = field.getHeader();
  EXPECT_NEAR(h.width,  p.resolution * static_cast<double>(p.cols), 1e-9);
  EXPECT_NEAR(h.height, p.resolution * static_cast<double>(p.rows), 1e-9);
}

// 全グリッドのrow/colインデックスが範囲内にあること
TEST_P(FieldSizeTest, AllGridRowColWithinBounds)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  auto* values = field.getValues();
  for (const auto& v : (*values))
  {
    EXPECT_LT(v.row, p.rows);
    EXPECT_LT(v.col, p.cols);
  }
}

// ============================================================
// 座標変換パラメータ化テスト（解像度ごと）
// ============================================================

struct FieldResolutionParams
{
  size_t rows;
  size_t cols;
  double resolution;
  double origin_x;
  double origin_y;
  // 検査する座標
  double query_x;
  double query_y;
  // 期待されるインデックス計算のため: col=(query_x - x_shift)/res, row=(query_y - y_shift)/res
  // x_shift = -cols/2*res + origin_x, y_shift = -rows/2*res + origin_y
};

class FieldResolutionTest : public ::testing::TestWithParam<FieldResolutionParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentResolutions,
  FieldResolutionTest,
  ::testing::Values(
    // 5x5, res=1.0, origin=(0,0): center -> index=12
    FieldResolutionParams{5, 5, 1.0, 0.0, 0.0, 0.0, 0.0},
    // 7x7, res=1.0, origin=(0,0): center -> index=24
    FieldResolutionParams{7, 7, 1.0, 0.0, 0.0, 0.0, 0.0},
    // 5x5, res=0.5, origin=(0,0): center -> index=12
    FieldResolutionParams{5, 5, 0.5, 0.0, 0.0, 0.0, 0.0},
    // 5x5, res=1.0, origin=(2,3): center -> index=12
    FieldResolutionParams{5, 5, 1.0, 2.0, 3.0, 2.0, 3.0}
  )
);

// origin座標でgetFieldIndexを呼んだとき中心付近のインデックスが返ること
TEST_P(FieldResolutionTest, GetFieldIndexReturnsExpectedForOriginPoint)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, p.origin_x, p.origin_y);
  // center index = (rows/2)*cols + (cols/2)
  size_t expected_center = (p.rows / 2) * p.cols + (p.cols / 2);
  size_t idx = field.getFieldIndex(p.query_x, p.query_y);
  EXPECT_EQ(idx, expected_center);
}

// getFieldCoordinate でインデックス→座標の逆変換が整合すること
TEST_P(FieldResolutionTest, GetFieldCoordinateConsistency)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, p.origin_x, p.origin_y);
  size_t center_idx = (p.rows / 2) * p.cols + (p.cols / 2);
  std::vector<double> coord = field.getFieldCoordinate(center_idx);
  ASSERT_EQ(coord.size(), 2u);
  // x_shift = -(cols/2)*res + origin_x
  double x_shift = -(static_cast<double>(p.cols) / 2.0) * p.resolution + p.origin_x;
  double y_shift = -(static_cast<double>(p.rows) / 2.0) * p.resolution + p.origin_y;
  double expected_x = static_cast<double>(p.cols / 2) * p.resolution + x_shift;
  double expected_y = static_cast<double>(p.rows / 2) * p.resolution + y_shift;
  EXPECT_NEAR(coord[0], expected_x, 1e-9);
  EXPECT_NEAR(coord[1], expected_y, 1e-9);
}

// ============================================================
// 境界値パラメータ化テスト
// ============================================================

struct FieldBoundaryParams
{
  size_t rows;
  size_t cols;
  double resolution;
  // アクセスするインデックス（境界セル）
  size_t boundary_index;
};

class FieldBoundaryTest : public ::testing::TestWithParam<FieldBoundaryParams> {};

INSTANTIATE_TEST_SUITE_P(
  BoundaryCells,
  FieldBoundaryTest,
  ::testing::Values(
    // 3x3: 先頭セル(0)・末尾セル(8)
    FieldBoundaryParams{3, 3, 1.0, 0},
    FieldBoundaryParams{3, 3, 1.0, 8},
    // 5x5: 先頭セル・末尾セル
    FieldBoundaryParams{5, 5, 1.0, 0},
    FieldBoundaryParams{5, 5, 1.0, 24},
    // 4x6: 先頭・末尾
    FieldBoundaryParams{4, 6, 0.5, 0},
    FieldBoundaryParams{4, 6, 0.5, 23}
  )
);

// 境界セルのgetValueが例外を投げないこと
TEST_P(FieldBoundaryTest, BoundaryCellAccessDoesNotThrow)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  EXPECT_NO_THROW({
    FieldGrid g = field.getValue(p.boundary_index);
    (void)g;
  });
}

// 境界セルのindexが正しく設定されていること
TEST_P(FieldBoundaryTest, BoundaryCellHasCorrectIndex)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  FieldGrid g = field.getValue(p.boundary_index);
  EXPECT_EQ(g.index, p.boundary_index);
}

// 範囲外インデックスは例外を投げること
TEST_P(FieldBoundaryTest, OutOfRangeIndexThrows)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  size_t out_of_range = p.rows * p.cols;  // 最大有効インデックス+1
  EXPECT_THROW(field.checkIndex(out_of_range), std::out_of_range);
}

// ============================================================
// setFieldInfo / searchFieldInfo パラメータ化テスト
// ============================================================

struct FieldInfoParams
{
  size_t rows;
  size_t cols;
  double resolution;
  GridInfo info_type;
  std::vector<size_t> set_indices;  // trueに設定するインデックス
};

class FieldInfoTest : public ::testing::TestWithParam<FieldInfoParams> {};

INSTANTIATE_TEST_SUITE_P(
  DifferentInfoTypes,
  FieldInfoTest,
  ::testing::Values(
    // 3x3でIS_OBSTACLEを複数設定
    FieldInfoParams{3, 3, 1.0, GridInfo::IS_OBSTACLE, {0, 2, 6, 8}},
    // 5x5でIS_GOALを1つ設定
    FieldInfoParams{5, 5, 1.0, GridInfo::IS_GOAL, {12}},
    // 7x7でIS_ROBOTを2つ設定
    FieldInfoParams{7, 7, 1.0, GridInfo::IS_ROBOT, {0, 48}},
    // 4x4でIS_PLANNED_PATHを3つ設定
    FieldInfoParams{4, 4, 0.5, GridInfo::IS_PLANNED_PATH, {1, 5, 10}}
  )
);

// setFieldInfoで設定したインデックスがsearchFieldInfoで正しく検出されること
TEST_P(FieldInfoTest, SetAndSearchFieldInfoMatchesCount)
{
  auto p = GetParam();
  Field field(p.rows, p.cols, p.resolution, 0.0, 0.0);
  for (size_t idx : p.set_indices)
  {
    field.setFieldInfo(idx, p.info_type, true);
  }
  std::vector<size_t> result;
  field.searchFieldInfo(result, p.info_type);
  EXPECT_EQ(result.size(), p.set_indices.size());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
