# T-016 チケット仕様書 — OptimalPathFollower・Interpolate の TEST_P パラメータ化テスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-016 |
| マイルストーン | M2 |
| タイトル | OptimalPathFollower・Interpolate の TEST_P パラメータ化テストを追加 |
| コミット | `372aba6` |
| ステータス | 完了 |

---

## 1. 実装内容

`OptimalPathFollower` および `Interpolate` クラスに対して `TEST_P` パラメータ化テストを追加した。

- `OptimalPathFollower`: 経路長・最適化手法・目標方向の各観点をパラメータ化し、到達判定・速度制限・コマンド出力を検証
- `Interpolate`: Bezier曲線（Vector2d版・Pose版）および線形補間の出力点数・制御点数を変えた多値検証

---

## 2. テストケース一覧

### 2.1 test_optimal_path_follower_param.cpp（合計 24 件）

#### OptimalPathFollowerPathLengthTest（6パラメータ × 2テスト関数 = 12件）

始点 (0,0) からゴール (goal_x, goal_y) まで n 分割した経路を `setTargetPath()` に設定し、`reachedTarget()` の結果を検証する。

| # | path_length | goal (x, y) | expected_reached | 説明 |
|---|---|---|---|---|
| 0 | 2 | (5, 0) | false | 最小経路（2点）、遠方ゴール |
| 1 | 5 | (5, 0) | false | 短い経路、遠方ゴール |
| 2 | 10 | (5, 0) | false | 中程度の経路、遠方ゴール |
| 3 | 20 | (5, 0) | false | 長い経路、遠方ゴール |
| 4 | 50 | (5, 0) | false | 非常に長い経路、遠方ゴール |
| 5 | 10 | (0.01, 0) | true | 近傍ゴール（stop_margin_distance_=0.03m 未満） |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `ReachedTargetMatchesExpectation` | `reachedTarget()` が expected_reached と一致 |
| `SetTargetPathNoThrow` | `setTargetPath()` が例外を投げない |

#### OptimalPathFollowerMethodTest（2パラメータ × 3テスト関数 = 6件）

最適化手法（`all_search` / `gradient`）ごとに `calculateCommand()` の速度出力が制限範囲内であることを検証する。

速度制限設定: `linear_velocity_min=-0.2`, `linear_velocity_max=0.2`, `angular_velocity_min=-1.0`, `angular_velocity_max=1.0`

| # | method | 説明 |
|---|---|---|
| 0 | `all_search` | 全探索最適化 |
| 1 | `gradient` | 勾配法最適化 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `LinearVelocityWithinLimit` | `follower.v ∈ [linear_velocity_min, linear_velocity_max]` |
| `AngularVelocityWithinLimit` | `follower.omega ∈ [angular_velocity_min, angular_velocity_max]` |
| `BestCmdWithinLimit` | `getBestCmd()` の返値が v・omega 両方ともリミット範囲内 |

#### OptimalPathFollowerDirectionTest（6パラメータ × 1テスト関数 = 6件）

様々な方向のゴールに対して `calculateCommand()` が非ゼロの速度コマンドを出力することを検証する。

| # | goal (x, y) | 説明 |
|---|---|---|
| 0 | (1, 0) | +X 方向 |
| 1 | (0, 1) | +Y 方向 |
| 2 | (-1, 0) | -X 方向 |
| 3 | (0, -1) | -Y 方向 |
| 4 | (1, 1) | 斜め（第一象限） |
| 5 | (-1, -1) | 斜め（第三象限） |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `ProducesCommandForDifferentDirections` | `abs(v) > 1e-9 または abs(omega) > 1e-9` |

---

### 2.2 test_interpolate_param.cpp（合計 68 件）

#### BezierVector2dNumPointsTest（7パラメータ × 2テスト関数 = 14件）

`bezier(Vector2d)` の出力点数（num_points）を変えてBezier補間を検証する。

制御点（固定）: `{(0,0), (1,2), (3,1), (4,3), (5,0)}`

| # | num_points | 説明 |
|---|---|---|
| 0 | 3 | 最小点数 |
| 1 | 5 | 少点数 |
| 2 | 10 | 標準点数 |
| 3 | 20 | 中点数 |
| 4 | 50 | 多点数 |
| 5 | 100 | 高密度 |
| 6 | 200 | 最高密度 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `OutputIsNotEmpty` | `bezier()` の出力が空でない |
| `StartPointNearFirstControlPoint` | `out.front()` が先頭制御点 `(1, 2)` と一致（許容 1e-6） |

#### BezierVector2dControlPointsTest（5パラメータ × 2テスト関数 = 10件）

`bezier(Vector2d)` の制御点数を変えて補間結果を検証する。制御点は直線上に均等配置。

| # | num_control_points | 説明 |
|---|---|---|
| 0 | 2 | 最小制御点 |
| 1 | 3 | 3制御点 |
| 2 | 5 | 5制御点 |
| 3 | 10 | 10制御点 |
| 4 | 20 | 20制御点 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `OutputIsNotEmpty` | 出力が空でない |
| `StartPointNearFirstControlPoint` | `out.front()` が先頭制御点と一致（許容 1e-6） |

#### LinearInterpolateTest（6パラメータ × 4テスト関数 = 24件）

`linear()` の出力点数を変えて線形補間を検証する。入力: `{(0,0), (5,5)}`。

| # | num_points | 説明 |
|---|---|---|
| 0 | 3 | 最小点数 |
| 1 | 5 | 少点数 |
| 2 | 10 | 標準点数 |
| 3 | 20 | 中点数 |
| 4 | 50 | 多点数 |
| 5 | 100 | 高密度 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `OutputIsNotEmpty` | `out.size() >= 2` |
| `StartPointPreserved` | `out.front()` が `(0, 0)` と一致 |
| `EndPointPreserved` | `out.back()` が `(5, 5)` と一致 |
| `AllPointsWithinBounds` | 全点が `[0, 5] x [0, 5]` の範囲内 |

#### BezierPoseTest（5パラメータ × 4テスト関数 = 20件）

`bezier(Pose)` の出力点数を変えて Pose 型 Bezier 補間を検証する。制御点: `{(0,0), (2,2), (4,0)}`。

| # | num_points | 説明 |
|---|---|---|
| 0 | 3 | 最小点数 |
| 1 | 5 | 少点数 |
| 2 | 10 | 標準点数 |
| 3 | 20 | 中点数 |
| 4 | 50 | 多点数 |

テスト関数:

| テスト関数名 | 検証内容 |
|---|---|
| `OutputIsNotEmpty` | 出力が空でない |
| `StartPointNearFirstControlPoint` | `out.front().position` が `(0, 0)` と一致（許容 1e-6） |
| `XCoordinateWithinBounds` | 全点の `position.x ∈ [-0.1, 4.1]` |
| `ConsistentWithVector2dVersion` | `bezier(Pose)` と `bezier(Vector2d)` が同一座標値を生成 |

---

## 3. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# OptimalPathFollower パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_optimal_path_follower_param

# Interpolate パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_interpolate_param

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 4. 対応ファイル

- `potbot_lib/test/test_optimal_path_follower_param.cpp` — OptimalPathFollower パラメータ化テスト
- `potbot_lib/test/test_interpolate_param.cpp` — Interpolate パラメータ化テスト
- `potbot_lib/CMakeLists.txt` — テスト登録設定
