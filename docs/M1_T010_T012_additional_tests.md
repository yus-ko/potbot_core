# M1: T-010〜T-012 追加テスト仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-010〜T-012 |
| マイルストーン | M1 |
| タイトル | test_apf_path_planner / optimal_path_follower / interpolate / utility にテストを追加 |
| コミット | `0ce9926` |
| ステータス | 完了 |

### 目的

T-001〜T-009 では未カバーだった境界値・品質ベンチマーク・冪等性・方向別動作確認などのテストを 4 ファイルに追加し、テストカバレッジをさらに向上させる。

### 背景

T-001〜T-009 で基本テストとエッジケーステストを追加した後、以下の観点でテストが不足していることが判明した：
- `APFPathPlanner` の経路品質・冪等性・Y軸方向・単調収束
- `OptimalPathFollower` の Y軸/斜め方向目標・速度ゼロ制限・収束シミュレーション・getSplitPath
- `Interpolate` の補間点境界チェック・spline サイズ上限・大量点 bezier・Pose 位置精度
- `Utility` の各種未テスト関数（get_vec/get_index 等）はすでに T-003 で対応済みのため T-012 は T-003 と同一コミットの追加分を含む

---

## 2. 実装内容

- `potbot_lib/test/test_apf_path_planner.cpp` に 5 件追加（M0: 11件 → M1: 16件）
- `potbot_lib/test/test_optimal_path_follower.cpp` に追加分を含む（T-001 と同コミットで完成）
- `potbot_lib/test/test_interpolate.cpp` に追加分を含む（T-002 と同コミットで完成）
- `potbot_lib/test/test_utility.cpp` に追加分を含む（T-003 と同コミットで完成）

---

## 3. テストケース一覧（T-010: ApfPathPlanner 追加 5 件）

### テストスイート: `APFPathPlannerTest`（追加分）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `PathLengthNearEuclideanDistance` | 障害物なしで経路長がユークリッド距離の 1.5 倍以内であること |
| 2 | `CreatePathIdempotent` | `createPath()` を 2 回連続で呼んでも 2 回目の結果が有効であること（冪等性） |
| 3 | `RobotEqualsGoalNocrash` | Robot と Goal が同じ座標 (0,0) でも `createPath()` がクラッシュしないこと |
| 4 | `CreatePathYDirection` | Robot=(0,-0.9), Goal=(0,0.9) で Y 軸方向の経路が生成されること |
| 5 | `PathMonotonicallyApproachesGoal` | 障害物なしで経路の 70% 以上の点がゴールに単調に近づいていること |

**T-010 追加テスト合計: 5 件（M0: 11件 → M1: 16件）**

---

## 4. T-011: OptimalPathFollower 追加テスト

T-001 のコミット（`b0e5087`）で test_optimal_path_follower.cpp が新規作成され、全 16 件のテストが実装された。T-011 はそのうち以下の追加観点に相当するテストを含む：

| テストケース名 | テスト内容 |
|---|---|
| `YDirectionTarget` | Y 軸方向目標への制御コマンド出力確認 |
| `DiagonalTarget` | 斜め方向目標への制御コマンド出力確認 |
| `ZeroLimitProducesZeroVelocity` | setLimit(0,0,0,0) 時に v=omega=0 になること |
| `ConvergenceSimulation` | 最大 500 ステップで reachedTarget() まで収束すること |
| `GetSplitPathNonEmpty` | calculateCommand() 後に getSplitPath() が空でないこと |

（これらは T-001 の test_optimal_path_follower.cpp に含まれる）

---

## 5. T-012: Interpolate / Utility 追加テスト

T-002 のコミット（`eb99eff`）で test_interpolate.cpp が新規作成され、全 17 件のテストが実装された。T-012 はそのうち以下の追加観点に相当するテストを含む：

| テストケース名 | テスト内容 |
|---|---|
| `LinearPointsWithinBounds` | 線形補間結果が全点 [0,5]×[0,5] 範囲内に収まること |
| `SplineSizeAtMostNumPoints` | スプライン補間結果が num_points=20 以下になること |
| `BezierVector2dLargeNumPoints` | num_points=200 の大量点 bezier が空でないこと |
| `BezierPosePreservesXY` | Pose ベースの bezier が Vector2d ベースと同一の位置成分を返すこと |
| `BezierPosePositionAccuracy` | Bezier 補間後の Pose X 座標が [0, 4] 範囲内に収まること |

（これらは T-002 の test_interpolate.cpp に含まれる）

T-003 のコミット（`e04773a`）で test_utility.cpp の Utility 未テスト関数テスト 22 件が追加された（詳細は T-003 仕様書を参照）。

---

## 6. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# 各ファイルのテストのみ実行
ctest --test-dir build/potbot_lib -R test_apf_path_planner
ctest --test-dir build/potbot_lib -R test_optimal_path_follower
ctest --test-dir build/potbot_lib -R test_interpolate
ctest --test-dir build/potbot_lib -R test_utility
```

---

## 7. 対応ファイル

- `potbot_lib/test/test_apf_path_planner.cpp` — 5 件追加（T-010）
- `potbot_lib/test/test_optimal_path_follower.cpp` — T-001 で完成（T-011 相当含む）
- `potbot_lib/test/test_interpolate.cpp` — T-002 で完成（T-012 相当含む）
- `potbot_lib/test/test_utility.cpp` — T-003 で追加（T-012 相当含む）
