# T-014 チケット仕様書 — Field・ArtificialPotentialField の TEST_P パラメータ化テスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-014 |
| マイルストーン | M2 |
| タイトル | Field・ArtificialPotentialField の TEST_P パラメータ化テストを追加 |
| コミット | `b877741` |
| ステータス | 完了 |

---

## 1. 実装内容

`Field` および `ArtificialPotentialField` クラスに対して `TEST_P` パラメータ化テストを追加した。

- `Field`: グリッドサイズ・解像度・境界インデックス・フィールド情報（IS_OBSTACLE 等）の各観点を複数パラメータセットで検証
- `ArtificialPotentialField`: ゲイン組み合わせ・引力ゲインスケール比較・障害物距離（dtr）閾値・ゴール位置・複数障害物の各観点を検証

---

## 2. テストケース一覧

### 2.1 test_field_param.cpp（合計 50 件）

#### FieldSizeTest（5パラメータ × 4テスト関数 = 20件）

`Field` コンストラクタに異なる rows/cols/resolution を与え、グリッドサイズの整合性を検証する。

| # | rows | cols | resolution | expected_total_cells | 説明 |
|---|---|---|---|---|---|
| 0 | 11 | 11 | 1.0 | 121 | 標準サイズ |
| 1 | 21 | 21 | 0.5 | 441 | 中解像度 |
| 2 | 41 | 41 | 0.05 | 1681 | 高解像度 |
| 3 | 5 | 10 | 1.0 | 50 | 非正方形 |
| 4 | 101 | 101 | 0.1 | 10201 | 大サイズ |

テスト関数（各パラメータセットで実行）:

| テスト関数名 | 検証内容 |
|---|---|
| `CellCountMatchesDimensions` | `getValues()->size() == rows * cols` |
| `HeaderRowsColsMatch` | `getHeader()` の rows・cols・resolution が一致 |
| `HeaderWidthHeightMatchResolutionTimesSize` | `width = resolution * cols`、`height = resolution * rows` |
| `AllGridRowColWithinBounds` | 全グリッドの row < rows かつ col < cols |

#### FieldResolutionTest（4パラメータ × 2テスト関数 = 8件）

座標からインデックスへの変換（`getFieldIndex`）と逆変換（`getFieldCoordinate`）の整合性を検証する。

| # | rows | cols | resolution | origin (x, y) | query (x, y) | 説明 |
|---|---|---|---|---|---|---|
| 0 | 5 | 5 | 1.0 | (0, 0) | (0, 0) | 中心インデックス=12 |
| 1 | 7 | 7 | 1.0 | (0, 0) | (0, 0) | 中心インデックス=24 |
| 2 | 5 | 5 | 0.5 | (0, 0) | (0, 0) | 解像度 0.5 で中心 |
| 3 | 5 | 5 | 1.0 | (2, 3) | (2, 3) | オフセット付き origin |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `GetFieldIndexReturnsExpectedForOriginPoint` | `getFieldIndex(query_x, query_y)` が中心インデックスを返す |
| `GetFieldCoordinateConsistency` | `getFieldCoordinate(center_idx)` が期待座標と一致 |

#### FieldBoundaryTest（6パラメータ × 3テスト関数 = 18件）

境界セルへのアクセスおよび範囲外インデックスの例外送出を検証する。

| # | rows | cols | resolution | boundary_index | 説明 |
|---|---|---|---|---|---|
| 0 | 3 | 3 | 1.0 | 0 | 3x3 先頭セル |
| 1 | 3 | 3 | 1.0 | 8 | 3x3 末尾セル |
| 2 | 5 | 5 | 1.0 | 0 | 5x5 先頭セル |
| 3 | 5 | 5 | 1.0 | 24 | 5x5 末尾セル |
| 4 | 4 | 6 | 0.5 | 0 | 4x6 先頭セル |
| 5 | 4 | 6 | 0.5 | 23 | 4x6 末尾セル |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `BoundaryCellAccessDoesNotThrow` | `getValue(boundary_index)` が例外を投げない |
| `BoundaryCellHasCorrectIndex` | 取得した `FieldGrid.index` が boundary_index と一致 |
| `OutOfRangeIndexThrows` | `checkIndex(rows*cols)` が `std::out_of_range` を投げる |

#### FieldInfoTest（4パラメータ × 1テスト関数 = 4件）

`setFieldInfo` で設定したグリッド情報が `searchFieldInfo` で正しく検索されることを検証する。

| # | rows | cols | resolution | info_type | set_indices | 説明 |
|---|---|---|---|---|---|---|
| 0 | 3 | 3 | 1.0 | IS_OBSTACLE | {0, 2, 6, 8} | 複数障害物セル設定 |
| 1 | 5 | 5 | 1.0 | IS_GOAL | {12} | ゴールセル設定 |
| 2 | 7 | 7 | 1.0 | IS_ROBOT | {0, 48} | ロボットセル設定 |
| 3 | 4 | 4 | 0.5 | IS_PLANNED_PATH | {1, 5, 10} | 計画経路セル設定 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `SetAndSearchFieldInfoMatchesCount` | `searchFieldInfo` の結果件数が set_indices の件数と一致 |

---

### 2.2 test_apf_param.cpp（合計 37 件）

#### APFGainTest（4パラメータ × 3テスト関数 = 12件）

APF ゲイン（引力・斥力・dtr）の各組み合わせでフィールド生成の正常動作を検証する。

| # | attr_gain | rep_gain | dtr | 説明 |
|---|---|---|---|---|
| 0 | 1.0 | 1.0 | 0.5 | 標準 |
| 1 | 2.0 | 0.5 | 0.3 | 高引力・低斥力 |
| 2 | 0.5 | 2.0 | 1.0 | 低引力・高斥力 |
| 3 | 0.1 | 0.1 | 0.1 | 低ゲイン全般 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `PotentialFieldIsGeneratedWithoutCrash` | `createPotentialField()` が例外を投げない |
| `GridSizeIsCorrect` | `getValues()->size() == 121`（11×11） |
| `TotalPotentialEqualsAttractionPlusRepulsion` | 全セルで `potential = attraction + repulsion` |

#### APFAttrGainScaleTest（3パラメータ × 1テスト関数 = 3件）

引力ゲインが高い APF の方が、ゴールから遠い場所でより大きな吸引ポテンシャルを持つことを検証する。

| # | attr_gain_low | attr_gain_high | rep_gain | dtr | 説明 |
|---|---|---|---|---|---|
| 0 | 1.0 | 2.0 | 0.0 | 100.0 | ゲイン 1.0 vs 2.0 |
| 1 | 0.5 | 1.5 | 0.0 | 100.0 | ゲイン 0.5 vs 1.5 |
| 2 | 0.1 | 1.0 | 0.0 | 100.0 | ゲイン 0.1 vs 1.0 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `HigherAttrGainProducesLargerAttraction` | 距離 > 2.0 の全セルで高ゲインの attraction > 低ゲインの attraction |

#### APFDtrTest（3パラメータ × 2テスト関数 = 6件）

斥力が効く距離閾値（dtr）の境界動作を検証する。

| # | dtr | obstacle (x, y) | far_dist | 説明 |
|---|---|---|---|---|
| 0 | 1.0 | (0, 0) | 2.0 | 距離 2.0 以上は斥力ゼロ |
| 1 | 2.0 | (0, 0) | 3.0 | 距離 3.0 以上は斥力ゼロ |
| 2 | 3.0 | (0, 0) | 4.0 | 距離 4.0 以上は斥力ゼロ |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `RepulsionIsZeroBeyondDtr` | `dist > far_dist` の全セルで `repulsion == 0.0` |
| `RepulsionExistsWithinDtr` | `0 < dist < dtr` の範囲で最大斥力 > 0 |

#### APFGoalTest（5パラメータ × 2テスト関数 = 10件）

ゴール位置の設定・取得と、ゴール付近のポテンシャル最小化を検証する。

| # | goal (x, y) | 説明 |
|---|---|---|
| 0 | (0, 0) | 中心 |
| 1 | (2, 0) | 右 |
| 2 | (-2, 0) | 左 |
| 3 | (0, 2) | 上 |
| 4 | (0, -2) | 下 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `SetGoalReturnsCorrectPosition` | `getGoal()` が設定値と一致 |
| `PotentialMinimumNearGoal` | ゴール付近（dist < 1.5）の最小ポテンシャル < 遠方（dist > 3.0）の最大ポテンシャル |

#### APFMultiObstacleTest（3パラメータ × 2テスト関数 = 6件）

複数障害物の設定・取得・クリアを検証する。

| # | rep_gain | dtr | obstacles | expected_count | 説明 |
|---|---|---|---|---|---|
| 0 | 1.0 | 2.0 | {(-3,0), (0,0), (3,0)} | 3 | X軸上3障害物 |
| 1 | 0.5 | 1.5 | {(-2,-2), (2,2)} | 2 | 対角2障害物 |
| 2 | 2.0 | 3.0 | {(-1,0)} | 1 | 1障害物 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `ObstacleCountMatches` | `getObstacles()` の件数が expected_count と一致 |
| `ClearObstaclesEmptiesList` | `clearObstacles()` 後に `getObstacles()` が空 |

---

## 3. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# Field パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_field_param

# ArtificialPotentialField パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_apf_param

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 4. 対応ファイル

- `potbot_lib/test/test_field_param.cpp` — Field パラメータ化テスト
- `potbot_lib/test/test_apf_param.cpp` — ArtificialPotentialField パラメータ化テスト
- `potbot_lib/CMakeLists.txt` — テスト登録設定
