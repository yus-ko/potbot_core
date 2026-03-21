# T-015 チケット仕様書 — ApfPathPlanner の TEST_P パラメータ化テスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-015 |
| マイルストーン | M2 |
| タイトル | ApfPathPlanner の TEST_P パラメータ化テストを追加 |
| コミット | `9d9c9d4` |
| ステータス | 完了 |

---

## 1. 実装内容

`APFPathPlanner` クラスに対して `TEST_P` パラメータ化テストを追加した。

異なるグリッドサイズ・解像度・スタート位置・ゴール位置の9通りの組み合わせに対して、経路生成の成否・始点の位置精度・終点の位置精度を検証する。

テスト設計上の重要な制約として、`createPath()` 内部に `norm() > 0.1` の距離チェックがあるため、グリッド解像度は対角距離 `res * sqrt(2) < 0.1m` を満たす `0.05m` 以下に設定している。

---

## 2. テストケース一覧

### 2.1 test_apf_path_planner_param.cpp（合計 27 件）

#### ApfPathPlannerParamTest（9パラメータ × 3テスト関数 = 27件）

APF フィールドを障害物なし（weight_rep=0.0）で生成し、各方向・距離への経路計画を検証する。

| # | rows | cols | resolution | start (x, y) | goal (x, y) | 説明 |
|---|---|---|---|---|---|---|
| 0 | 41 | 41 | 0.05 | (0, 0) | (0.5, 0.5) | 高解像度グリッド正方向（斜め移動） |
| 1 | 41 | 41 | 0.05 | (0, 0) | (0, 0.5) | Y 軸方向移動 |
| 2 | 41 | 41 | 0.05 | (0, 0) | (0.5, 0) | X 軸方向移動 |
| 3 | 41 | 41 | 0.05 | (-0.3, -0.3) | (0.3, 0.3) | 負座標から正座標（斜め移動） |
| 4 | 41 | 41 | 0.05 | (0, 0) | (-0.3, 0.3) | 負方向ゴール（Y軸正、X軸負） |
| 5 | 51 | 51 | 0.04 | (-0.8, 0) | (0.8, 0) | 中グリッド X 軸長距離移動 |
| 6 | 81 | 81 | 0.025 | (0, -0.9) | (0, 0.9) | 大グリッド Y 軸長距離移動 |
| 7 | 41 | 41 | 0.05 | (0, 0) | (0.1, 0) | 短距離 X 軸移動 |
| 8 | 41 | 41 | 0.05 | (-0.5, -0.5) | (-0.1, -0.1) | 負座標領域内移動 |

**APF 設定（全ケース共通）**: `weight_attr=1.0`, `weight_rep=0.0`, `dtr=10.0`, `origin=(0,0)`

**プランナー設定**: `planner.setParams(3.0, 1, 1.0, 0.0)`、`createPath(0.0)`

テスト関数:

| テスト関数名 | 検証内容 | 許容誤差 |
|---|---|---|
| `CreatePathSucceeds` | `createPath()` が true を返す | — |
| `PathStartsNearRobot` | `path[0]` がロボット位置から `resolution * 2.0` m 以内 | `resolution × 2.0` |
| `PathEndsNearGoal` | `path.back()` がゴール位置から 0.3 m 以内 | 0.3 m（6グリッド相当 @ res=0.05m） |

---

## 3. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# ApfPathPlanner パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_apf_path_planner_param

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 4. 対応ファイル

- `potbot_lib/test/test_apf_path_planner_param.cpp` — ApfPathPlanner パラメータ化テスト
- `potbot_lib/CMakeLists.txt` — テスト登録設定
