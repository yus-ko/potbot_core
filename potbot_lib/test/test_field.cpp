#include <gtest/gtest.h>
#include <potbot_lib/field.hpp>
#include <stdexcept>

using namespace potbot_lib;
using namespace potbot_lib::potential;

// ============================================================
// Field 初期化テスト
// ============================================================

TEST(FieldTest, DefaultConstructor)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    FieldHeader h = field.getHeader();
    EXPECT_EQ(h.rows, 3u);
    EXPECT_EQ(h.cols, 3u);
    EXPECT_DOUBLE_EQ(h.resolution, 1.0);
}

TEST(FieldTest, GridCount)
{
    Field field(4, 5, 1.0, 0.0, 0.0);
    auto* values = field.getValues();
    EXPECT_EQ(values->size(), 20u);  // 4*5
}

TEST(FieldTest, GridIndexing)
{
    // 3x3 grid, resolution=1.0, origin=(0,0)
    // x_shift = -3/2 = -1.5, y_shift = -3/2 = -1.5
    // col0->x=-1.5, col1->x=-0.5, col2->x=0.5
    // row0->y=-1.5, row1->y=-0.5, row2->y=0.5
    Field field(3, 3, 1.0, 0.0, 0.0);
    auto* values = field.getValues();
    EXPECT_EQ((*values)[0].row, 0u);
    EXPECT_EQ((*values)[0].col, 0u);
    EXPECT_EQ((*values)[4].row, 1u);  // 中心
    EXPECT_EQ((*values)[4].col, 1u);
    EXPECT_EQ((*values)[8].row, 2u);
    EXPECT_EQ((*values)[8].col, 2u);
}

TEST(FieldTest, GridCoordinates)
{
    // 3x3 grid, resolution=1.0, origin=(0,0)
    Field field(3, 3, 1.0, 0.0, 0.0);
    // x_shift = -1.5, y_shift = -1.5
    // (row=0, col=0) -> x=-1.5, y=-1.5
    // (row=1, col=1) -> x=-0.5, y=-0.5
    // (row=2, col=2) -> x= 0.5, y= 0.5
    auto* values = field.getValues();
    EXPECT_NEAR((*values)[0].x, -1.5, 1e-9);
    EXPECT_NEAR((*values)[0].y, -1.5, 1e-9);
    EXPECT_NEAR((*values)[4].x, -0.5, 1e-9);
    EXPECT_NEAR((*values)[4].y, -0.5, 1e-9);
    EXPECT_NEAR((*values)[8].x, 0.5, 1e-9);
    EXPECT_NEAR((*values)[8].y, 0.5, 1e-9);
}

// ============================================================
// getFieldIndex() テスト
// ============================================================

TEST(FieldTest, GetFieldIndexCenter)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    // row=1, col=1 -> index=4 (center)
    size_t idx = field.getFieldIndex(static_cast<size_t>(1), static_cast<size_t>(1));
    EXPECT_EQ(idx, 4u);
}

TEST(FieldTest, GetFieldIndexByCoordinate)
{
    Field field(5, 5, 1.0, 0.0, 0.0);
    // origin=(0,0): x_shift=-2.5, y_shift=-2.5
    // 原点(0,0) に最も近いグリッドのインデックスを取得
    // col = (0.0 - (-2.5))/1.0 = 2, row = 2, idx = 2*5+2 = 12
    size_t idx = field.getFieldIndex(0.0, 0.0);
    EXPECT_EQ(idx, 12u);
}

TEST(FieldTest, GetFieldIndexOutOfRangeThrows)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    // フィールド外の座標は例外を投げる
    EXPECT_THROW(field.getFieldIndex(100.0, 100.0), std::out_of_range);
}

TEST(FieldTest, CheckIndexOutOfRangeThrows)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    EXPECT_THROW(field.checkIndex(100), std::out_of_range);
}

// ============================================================
// setFieldInfo() / searchFieldInfo() テスト
// ============================================================

TEST(FieldTest, SetAndSearchFieldInfo)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    // index=4 (中心) を IS_GOAL に設定
    field.setFieldInfo(4, GridInfo::IS_GOAL, true);

    std::vector<size_t> result;
    field.searchFieldInfo(result, GridInfo::IS_GOAL);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0], 4u);
}

TEST(FieldTest, SetFieldInfoFalse)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    field.setFieldInfo(0, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(0, GridInfo::IS_OBSTACLE, false);

    std::vector<size_t> result;
    field.searchFieldInfo(result, GridInfo::IS_OBSTACLE);
    EXPECT_EQ(result.size(), 0u);
}

TEST(FieldTest, SearchFieldInfoMultipleMatches)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    field.setFieldInfo(0, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(2, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(6, GridInfo::IS_OBSTACLE, true);

    std::vector<size_t> result;
    field.searchFieldInfo(result, GridInfo::IS_OBSTACLE);
    EXPECT_EQ(result.size(), 3u);
}

TEST(FieldTest, SearchFieldInfoAndMode)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    // index=4に IS_OBSTACLE と IS_GOAL の両方を設定
    field.setFieldInfo(4, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(4, GridInfo::IS_GOAL, true);
    // index=0に IS_OBSTACLE のみ設定
    field.setFieldInfo(0, GridInfo::IS_OBSTACLE, true);

    std::vector<size_t> result;
    field.searchFieldInfo(result, {GridInfo::IS_OBSTACLE, GridInfo::IS_GOAL}, "and");
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0], 4u);
}

TEST(FieldTest, SearchFieldInfoOrMode)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    field.setFieldInfo(0, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(4, GridInfo::IS_GOAL, true);

    std::vector<size_t> result;
    field.searchFieldInfo(result, {GridInfo::IS_OBSTACLE, GridInfo::IS_GOAL}, "or");
    EXPECT_EQ(result.size(), 2u);
}

// ============================================================
// getSquareIndex() テスト
// ============================================================

TEST(FieldTest, GetSquareIndexCenter)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    std::vector<size_t> indexes;
    // 中心(row=1, col=1)の周囲インデックス(range=1)
    field.getSquareIndex(indexes, 1, 1, 1);
    // 9-1(center)=8個の周囲セル
    EXPECT_EQ(indexes.size(), 8u);
}

TEST(FieldTest, GetSquareIndexCorner)
{
    // コーナー(0,0)でrange=1の場合、フィールド内の有効なセルのみが返る。
    // 旧実装では size_t アンダーフローと全クリアバグにより空が返っていたが、
    // 修正後は (0,1), (1,0), (1,1) の3セルが返る。
    Field field(5, 5, 1.0, 0.0, 0.0);
    std::vector<size_t> indexes;
    field.getSquareIndex(indexes, 0, 0, 1);
    // (row=0,col=1)=1, (row=1,col=0)=5, (row=1,col=1)=6 の3セルが有効
    EXPECT_EQ(indexes.size(), 3u);
}

// ============================================================
// getValue() テスト
// ============================================================

TEST(FieldTest, GetValueByIndex)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    FieldGrid g = field.getValue(static_cast<size_t>(0));
    EXPECT_EQ(g.index, 0u);
}

TEST(FieldTest, GetValueByCoordinate)
{
    Field field(5, 5, 1.0, 0.0, 0.0);
    // origin=(0,0) の場合、(0,0)座標はindex=12
    FieldGrid g = field.getValue(0.0, 0.0);
    EXPECT_EQ(g.index, 12u);
}

// ============================================================
// infoFilter() テスト
// ============================================================

TEST(FieldTest, InfoFilter)
{
    Field field(3, 3, 1.0, 0.0, 0.0);
    field.setFieldInfo(1, GridInfo::IS_OBSTACLE, true);
    field.setFieldInfo(5, GridInfo::IS_OBSTACLE, true);

    Field filtered;
    field.infoFilter(filtered, GridInfo::IS_OBSTACLE);
    auto* fv = filtered.getValues();
    EXPECT_EQ(fv->size(), 2u);
}

// ============================================================
// ヘッダー情報テスト
// ============================================================

TEST(FieldTest, HeaderBounds)
{
    // 5x5 grid, resolution=0.1, origin=(0,0)
    Field field(5, 5, 0.1, 0.0, 0.0);
    FieldHeader h = field.getHeader();
    EXPECT_NEAR(h.width, 0.5, 1e-9);    // 0.1*5
    EXPECT_NEAR(h.height, 0.5, 1e-9);   // 0.1*5
    EXPECT_NEAR(h.x_min, -0.25, 1e-9);  // -width/2
    EXPECT_NEAR(h.x_max, 0.25, 1e-9);   // width/2
    EXPECT_NEAR(h.y_min, -0.25, 1e-9);
    EXPECT_NEAR(h.y_max, 0.25, 1e-9);
}

TEST(FieldTest, HeaderWithNonZeroOrigin)
{
    Field field(4, 4, 1.0, 2.0, 3.0);
    FieldHeader h = field.getHeader();
    EXPECT_NEAR(h.x_min, 0.0, 1e-9);   // -4/2+2 = 0
    EXPECT_NEAR(h.x_max, 4.0, 1e-9);   // 4/2+2 = 4
    EXPECT_NEAR(h.y_min, 1.0, 1e-9);   // -4/2+3 = 1
    EXPECT_NEAR(h.y_max, 5.0, 1e-9);   // 4/2+3 = 5
}

// ============================================================
// setValue() テスト
// ============================================================

TEST(FieldTest, SetValueUpdatesGrid)
{
    // setValue() で特定インデックスの値を更新し getValue() で取得できること
    Field field(3, 3, 1.0, 0.0, 0.0);
    FieldGrid g;
    g.index = 4;       // 中心セル
    g.value = 99.0;
    field.setValue(g);
    FieldGrid result = field.getValue(static_cast<size_t>(4));
    EXPECT_EQ(result.index, 4u);
    EXPECT_DOUBLE_EQ(result.value, 99.0);
}

// ============================================================
// setValues() テスト
// ============================================================

TEST(FieldTest, SetValuesReplacesVector)
{
    // setValues() で新しいベクターを設定し getValues() で取得できること
    Field field(3, 3, 1.0, 0.0, 0.0);
    auto* original = field.getValues();
    std::vector<FieldGrid> newvals = *original;  // 同サイズのコピーを作成
    newvals[0].value = 42.0;
    newvals[8].value = 84.0;
    field.setValues(newvals);
    auto* updated = field.getValues();
    EXPECT_DOUBLE_EQ((*updated)[0].value, 42.0);
    EXPECT_DOUBLE_EQ((*updated)[8].value, 84.0);
}

// ============================================================
// getFieldCoordinate() テスト
// ============================================================

TEST(FieldTest, GetFieldCoordinateCenter)
{
    // 5x5フィールドのindex=12（中心）で getFieldCoordinate(12) が正しいx,y座標を返すこと
    // x_shift = -5/2 = -2.5, y_shift = -2.5
    // index=12 -> row=2, col=2 -> x = 2*1.0 + (-2.5) = -0.5, y = -0.5
    Field field(5, 5, 1.0, 0.0, 0.0);
    std::vector<double> coord = field.getFieldCoordinate(12);
    ASSERT_EQ(coord.size(), 2u);
    EXPECT_NEAR(coord[0], -0.5, 1e-9);  // x座標
    EXPECT_NEAR(coord[1], -0.5, 1e-9);  // y座標
}

// ============================================================
// setOrigin() テスト
// ============================================================

TEST(FieldTest, SetOriginAndGetFieldIndex)
{
    // setOrigin(1.0, 2.0) 後に getFieldIndex(1.0, 2.0) が中心付近のインデックスを返すこと
    // origin=(1.0, 2.0), 5x5, resolution=1.0
    // x_shift = -5/2 + 1.0 = -1.5, y_shift = -5/2 + 2.0 = -0.5
    // (1.0, 2.0) -> col = (1.0 - (-1.5))/1.0 = 2, row = (2.0 - (-0.5))/1.0 = 2, idx = 2*5+2 = 12
    Field field(5, 5, 1.0, 0.0, 0.0);
    field.setOrigin(1.0, 2.0);
    field.setHeader(5, 5, 1.0);
    field.initField();
    size_t idx = field.getFieldIndex(1.0, 2.0);
    EXPECT_EQ(idx, 12u);
}

// ============================================================
// getFieldIndex(Point) テスト
// ============================================================

TEST(FieldTest, GetFieldIndexByPoint)
{
    // getFieldIndex(Point(0.0, 0.0, 0.0)) が5x5フィールドのindex=12を返すこと
    // origin=(0,0), x_shift=-2.5, y_shift=-2.5
    // (0,0) -> col=2, row=2, idx=12
    Field field(5, 5, 1.0, 0.0, 0.0);
    Point p(0.0, 0.0, 0.0);
    size_t idx = field.getFieldIndex(p);
    EXPECT_EQ(idx, 12u);
}

// ============================================================
// setHeader() テスト
// ============================================================

TEST(FieldTest, SetHeaderUpdatesHeaderInfo)
{
    // setHeader(7, 7, 0.5) 後に getHeader() が rows=7, cols=7, resolution=0.5 を返すこと
    Field field(3, 3, 1.0, 0.0, 0.0);
    field.setHeader(7, 7, 0.5);
    FieldHeader h = field.getHeader();
    EXPECT_EQ(h.rows, 7u);
    EXPECT_EQ(h.cols, 7u);
    EXPECT_DOUBLE_EQ(h.resolution, 0.5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
