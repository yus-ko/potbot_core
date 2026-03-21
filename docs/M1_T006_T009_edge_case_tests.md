# M1: T-006〜T-009 エッジケーステスト仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-006〜T-009 |
| マイルストーン | M1 |
| タイトル | test_diff_drive_agent / pid / field / apf にエッジケーステストを追加 |
| コミット | `243d275` |
| ステータス | 完了 |

### 目的

M0 で作成した 4 クラスの既存テストに、境界値・エッジケースのテストを追加してカバレッジを向上させる。

### 背景

M0 のテストは各クラスの基本的な動作を検証していたが、後退移動・yaw 角の大きな累積・境界上の座標処理・複数障害物干渉など、実際のロボット動作で発生しうるエッジケースがカバーされていなかった。M1 にてこれらを補完した。

---

## 2. 実装内容

- `potbot_lib/test/test_diff_drive_agent.cpp` に 5 件追加（M0: 17件 → M1: 22件）
- `potbot_lib/test/test_pid.cpp` に 5 件追加（M0: 16件 → M1: 21件）
- `potbot_lib/test/test_field.cpp` に 6 件追加（M0: 20件 → M1: 26件）
- `potbot_lib/test/test_artificial_potential_field.cpp` に 6 件追加（M0: 19件 → M1: 25件）

---

## 3. テストケース一覧（追加分のみ）

### T-006: DiffDriveAgent エッジケーステスト（追加 5 件）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `UpdateBackward` | 後退移動: v=-1.0, yaw=0, dt=0.1 で update() 後に x=-0.1 になること |
| 2 | `UpdateYawAccumulationBeyondPi` | yaw 角の ±π 超え継続更新: omega=1.0, dt=0.1 で 20 回 update() 後に yaw≈2.0 [rad] になること |
| 3 | `UpdateDiagonal45Degrees` | 斜め 45 度方向移動精度: yaw=π/4, v=1.0, dt=0.1 で x≈y≈0.1/√2 になること |
| 4 | `GetAnglePoseYDirection` | `getAngle(Pose)` の Y 方向: ターゲット Pose が (0,1,0) の場合 getAngle=π/2 になること |
| 5 | `UpdateHighPrecisionMultipleSteps` | 複数ステップ後の位置精度: v=1.0, dt=0.01 で 100 回 update 後に x≈1.0（誤差 1e-5 以内）になること |

### T-007: PID エッジケーステスト（追加 5 件）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `ConvergesToYAxisTarget` | Y 軸方向目標 (0, 0.5) に対して 500 ステップ後にロボットが目標に近づいていること（距離が減少すること） |
| 2 | `ProcessBecomesStraightWhenFacingTarget` | target=(1,0,0) で複数回 calculateCommand() を呼ぶと PROCESS_STRAIGHT に遷移すること |
| 3 | `InitPIDResetsProcessToStop` | calculateCommand() 数回後に initPID() でプロセスが PROCESS_STOP にリセットされること |
| 4 | `ZeroGainProducesZeroVelocity` | PID ゲインがすべて 0 の場合は制御出力が 0 になり速度も 0 のままであること |
| 5 | `LargerDeltatimeConvergesFaster` | deltatime が大きいほど収束に必要なステップ数が少なくなること |

### T-008: Field エッジケーステスト（追加 6 件）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `SetValueUpdatesGrid` | `setValue()` で特定インデックスの値を更新し `getValue()` で取得できること |
| 2 | `SetValuesReplacesVector` | `setValues()` で新しいベクターを設定し `getValues()` で取得できること |
| 3 | `GetFieldCoordinateCenter` | 5×5 フィールドの index=12（中心）で `getFieldCoordinate(12)` が正しい x,y 座標を返すこと |
| 4 | `SetOriginAndGetFieldIndex` | `setOrigin(1.0, 2.0)` 後に `getFieldIndex(1.0, 2.0)` が中心付近のインデックス（12）を返すこと |
| 5 | `GetFieldIndexByPoint` | `getFieldIndex(Point(0.0, 0.0, 0.0))` が 5×5 フィールドの index=12 を返すこと |
| 6 | `SetHeaderUpdatesHeaderInfo` | `setHeader(7, 7, 0.5)` 後に `getHeader()` が rows=7, cols=7, resolution=0.5 を返すこと |

### T-009: ArtificialPotentialField エッジケーステスト（追加 6 件）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `MultipleObstaclesCreateRepulsionAtAllLocations` | 3 つの障害物を配置後、すべての障害物付近に斥力フィールドが存在すること |
| 2 | `ClearObstaclesThenRepulsionIsZero` | `clearObstacles()` 後に `createPotentialField()` を再実行すると全グリッドの斥力がゼロになること |
| 3 | `SetParamsWeightAttractionAffectsPotential` | wa=2.0 の吸引ポテンシャルが wa=1.0 より大きいこと |
| 4 | `InitPotentialFieldResetsToZero` | `createPotentialField()` 後に `initPotentialField()` を呼ぶと potential/attraction/repulsion が 0 にリセットされること |
| 5 | `RobotAndGoalSamePositionNoCrash` | Robot と Goal が同じ座標でも `createPotentialField()` がクラッシュしないこと |
| 6 | `SearchRepulsionFieldInsideDetected` | 障害物設置後に `IS_REPULSION_FIELD_INSIDE` が 1 つ以上検出されること |

**追加テスト合計: 22 件**

---

## 4. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# 各クラスのテストのみ実行
ctest --test-dir build/potbot_lib -R test_diff_drive_agent
ctest --test-dir build/potbot_lib -R test_pid
ctest --test-dir build/potbot_lib -R test_field
ctest --test-dir build/potbot_lib -R test_artificial_potential_field
```

---

## 5. 対応ファイル

- `potbot_lib/test/test_diff_drive_agent.cpp` — 5 件追加（T-006）
- `potbot_lib/test/test_pid.cpp` — 5 件追加（T-007）
- `potbot_lib/test/test_field.cpp` — 6 件追加（T-008）
- `potbot_lib/test/test_artificial_potential_field.cpp` — 6 件追加（T-009）
