# M1: T-002 test_interpolate 仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| マイルストーン | M1 |
| タイトル | test_interpolate を追加 |
| コミット | `eb99eff` |
| ステータス | 完了 |

### 目的

M0 のスコープ外だった `Interpolate` クラスに対して Google Test (gtest) ベースのユニットテストを新規作成し、Bezier 曲線・線形・スプライン補間の動作を自動的に検証できる体制を整える。

### 背景

`potbot_lib::interpolate` 名前空間に実装された補間関数群（linear/spline/bezier）は `ApfPathPlanner` の経路補間処理に使用されている。M0 では対象外とされていたが、M1 にて単体テストを整備した。

---

## 2. 実装内容

- `potbot_lib/test/test_interpolate.cpp` を新規作成（17 テストケース）
- `potbot_lib/CMakeLists.txt` に `test_interpolate` のテストターゲットを追加

---

## 3. テストケース一覧

### テストスイート: `InterpolateTest`

#### linear() テスト

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `LinearTwoPoints` | 2点入力で補間結果が 2 点以上になること |
| 2 | `LinearSinglePoint` | 1点入力では出力がそのまま 1 点になること |
| 3 | `LinearStartEndPreserved` | 始点・終点が補間結果に保存されること |
| 4 | `LinearPointsWithinBounds` | 補間結果の全点が入力点の座標範囲 [0,5]×[0,5] に収まること |

#### spline() テスト

| # | テストケース名 | テスト内容 |
|---|---|---|
| 5 | `SplineTwoPoints` | 2点入力でスプライン補間が例外なく実行され結果が 2 点以上になること |
| 6 | `SplineMultiplePoints` | 4点入力で補間結果が 2 点以上になること |
| 7 | `SplineSinglePoint` | 1点入力では出力がそのまま 1 点になること |
| 8 | `SplineSizeAtMostNumPoints` | 4点入力・num_points=20 の結果が 20 点以下 2 点以上になること |

#### bezier(Vector2d) テスト

| # | テストケース名 | テスト内容 |
|---|---|---|
| 9 | `BezierVector2dTwoPoints` | 2点入力で Bezier 補間結果が空でないこと |
| 10 | `BezierVector2dSinglePoint` | 1点入力では出力がそのまま 1 点になること |
| 11 | `BezierVector2dStartNearFirstPoint` | Bezier 補間の始点が入力の最初の点付近になること |
| 12 | `BezierVector2dEndNearLastPoint` | Bezier 補間の終点が入力の最後の点付近（誤差 0.1 以内）になること |
| 13 | `BezierVector2dLargeNumPoints` | 5点入力・num_points=200 で補間結果が空でないこと |

#### bezier(Pose) テスト

| # | テストケース名 | テスト内容 |
|---|---|---|
| 14 | `BezierPoseTwoPoints` | 2点 Pose 入力で Bezier 補間結果が空でないこと |
| 15 | `BezierPosePreservesXY` | Pose ベースと Vector2d ベースの bezier() が同一の位置成分を返すこと |
| 16 | `BezierPosePositionAccuracy` | Bezier 補間後の Pose の X 座標が [0, 4] 範囲内に収まること |

#### bezier(Point) テスト

| # | テストケース名 | テスト内容 |
|---|---|---|
| 17 | `BezierPointEmptyOrPassthrough` | Point 版 bezier() が例外なく動作し、空実装のため出力が空であること |

**合計: 17 件**

---

## 4. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# このファイルのみ実行
ctest --test-dir build/potbot_lib -R test_interpolate
```

---

## 5. 対応ファイル

- `potbot_lib/test/test_interpolate.cpp` — テスト本体（新規作成）
- `potbot_lib/CMakeLists.txt` — テストターゲット追加
