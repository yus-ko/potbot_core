# T-018 チケット仕様書 — リグレッションテスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-018 |
| マイルストーン | M2 |
| タイトル | リグレッションテストを追加（既知エッジケースの再発防止） |
| コミット | `514a63b` |
| ステータス | 完了 |

---

## 1. 実装内容

`potbot_lib/test/regression/` ディレクトリに既知エッジケースのリグレッションテストを作成した。

Navigation 2 のリグレッションテストパターン（nav2_costmap_2d/test/regression/）に準拠し、開発中に発見・対処したエッジケースをテストとして記録する。各テストには背景（なぜこのテストが必要か）・テスト由来（対応するユニットテスト名）を明記し、将来の開発者がリグレッションの文脈を理解できるようにした。

---

## 2. テストケース一覧

### 2.1 regression/test_regression.cpp（合計 15 件）

#### DiffDriveAgentRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `YawAccumulationNoNanInf` | REG-001 | omega=10.0, dt=0.02 で1000回 update() を連続実行してもオーバーフローしないことを確認 | yaw・x・y が Inf/NaN にならない |
| `HighSpeedRotationNoThrow` | REG-002 | omega=100.0 という極端な値でも100回 update() が例外なく完了することを確認 | `EXPECT_NO_THROW` で100回 update() が完了 |
| `BackwardMotionNoNanInf` | REG-003 | v=-1.0（後退）での1000回 update() で数値が安定することを確認 | x・y が Inf/NaN にならない |

#### FieldRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `OutOfRangeThrowsException` | REG-004 | `getFieldIndex(100,100)` と `checkIndex(100)` が `std::out_of_range` を投げてメモリ安全性を保つことを確認 | `std::out_of_range` が投げられる |
| `MinimalGridNoCrash` | REG-005 | rows=3, cols=3 という最小サイズのグリッドでも初期化がクラッシュしないことを確認 | `EXPECT_NO_THROW`、`getValues()->size() == 9` |
| `InfoFilterOnEmptyFlagsNoCrash` | REG-015 | IS_OBSTACLE が1つも設定されていない状態で `infoFilter()` を呼んでもクラッシュしないことを確認 | `EXPECT_NO_THROW`、フィルタ結果が空 |

#### ApfPathPlannerRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `RobotAtGoalNoCrash` | REG-006 | 起点=終点 (0,0) の場合に `createPath()` がクラッシュしないことを確認（ゼロ距離の除算・無限ループ防止） | `EXPECT_NO_THROW` で `createPath()` が完了 |
| `BezierOnEmptyPathNoCrash` | REG-007 | `createPath()` 前に `bezier()` を呼んでも空コンテナアクセスによる未定義動作が発生しないことを確認 | `EXPECT_NO_THROW`、`bezier()` が false を返す |

#### APFRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `RobotAndGoalSamePositionNoCrash` | REG-008 | ロボット=ゴール位置 (0,0) でも `createPotentialField()` がクラッシュしないことを確認（ゼロ距離でのポテンシャル計算における除算エラー防止） | `EXPECT_NO_THROW` で `createPotentialField()` が完了 |
| `CreatePotentialFieldWithoutGoalNoCrash` | REG-009 | `setGoal()` を呼ばずに `createPotentialField()` を実行してもクラッシュしないことを確認 | `EXPECT_NO_THROW` で `createPotentialField()` が完了 |
| `NoObstacleRepulsionIsZero` | REG-010 | `setObstacle()` を呼ばない状態で `createPotentialField()` を実行しても全グリッドの斥力がゼロであることを確認 | 全グリッドで `repulsion == 0.0` |

#### OptimalPathFollowerRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `EmptyPathReachedTargetNoCrash` | REG-011 | `setTargetPath()` 未呼び出し状態で `reachedTarget()` を呼んでもセグメンテーションフォールトが発生しないことを確認 | `EXPECT_NO_THROW`、`reachedTarget() == true` |
| `ZeroLimitProducesZeroVelocity` | REG-012 | linear_velocity_max=0, linear_velocity_min=0 に設定後 `calculateCommand()` を呼んでも v=0, omega=0 を維持することを確認 | `v ≈ 0.0`、`omega ≈ 0.0`（許容 1e-9） |

#### PIDRegression

| テスト名 | テスト ID | 背景 | 検証内容 |
|---|---|---|---|
| `InitPIDResetsProcessToStop` | REG-013 | `calculateCommand()` でプロセスが遷移した後に `initPID()` を呼ぶと `PROCESS_STOP` にリセットされることを確認（状態リセット漏れによる制御ループ再起動時の誤動作防止） | `getCurrentProcess() == PROCESS_STOP`、`v ≈ 0.0`、`omega ≈ 0.0` |
| `ZeroGainProducesZeroVelocity` | REG-014 | PID ゲインが全て 0 の場合に `calculateCommand()` が v=0, omega=0 を出力することを確認 | `v ≈ 0.0`、`omega ≈ 0.0`（許容 1e-9） |

---

## 3. リグレッション ID と対応テスト関数の対照表

| REG-ID | テストスイート | テスト関数名 | 対応するユニットテスト由来 |
|---|---|---|---|
| REG-001 | DiffDriveAgentRegression | YawAccumulationNoNanInf | — |
| REG-002 | DiffDriveAgentRegression | HighSpeedRotationNoThrow | UpdateYawAccumulationBeyondPi |
| REG-003 | DiffDriveAgentRegression | BackwardMotionNoNanInf | UpdateBackward |
| REG-004 | FieldRegression | OutOfRangeThrowsException | GetFieldIndexOutOfRangeThrows, CheckIndexOutOfRangeThrows |
| REG-005 | FieldRegression | MinimalGridNoCrash | — |
| REG-006 | ApfPathPlannerRegression | RobotAtGoalNoCrash | RobotEqualsGoalNocrash |
| REG-007 | ApfPathPlannerRegression | BezierOnEmptyPathNoCrash | BezierOnEmptyPathReturnsFalse |
| REG-008 | APFRegression | RobotAndGoalSamePositionNoCrash | RobotAndGoalSamePositionNoCrash |
| REG-009 | APFRegression | CreatePotentialFieldWithoutGoalNoCrash | — |
| REG-010 | APFRegression | NoObstacleRepulsionIsZero | ClearObstaclesThenRepulsionIsZero |
| REG-011 | OptimalPathFollowerRegression | EmptyPathReachedTargetNoCrash | EmptyPathReachedTarget |
| REG-012 | OptimalPathFollowerRegression | ZeroLimitProducesZeroVelocity | ZeroLimitProducesZeroVelocity |
| REG-013 | PIDRegression | InitPIDResetsProcessToStop | InitPIDResetsProcess |
| REG-014 | PIDRegression | ZeroGainProducesZeroVelocity | ZeroGainProducesZeroVelocity |
| REG-015 | FieldRegression | InfoFilterOnEmptyFlagsNoCrash | — |

---

## 4. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# リグレッションテストのみ実行
ctest --test-dir build/potbot_lib -R test_regression

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 5. 対応ファイル

- `potbot_lib/test/regression/test_regression.cpp` — リグレッションテスト実装（全 15 件）
- `potbot_lib/test/regression/CMakeLists.txt` — リグレッションテスト用ビルド設定
- `potbot_lib/CMakeLists.txt` — テスト登録設定
