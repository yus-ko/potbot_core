# M0: T-001 potbot_lib ユニットテスト仕様書

## 1. チケット概要

### 目的

`potbot_lib` のコアアルゴリズムクラス群に対して Google Test (gtest) を用いたユニットテストを新規作成し、各クラスの動作を自動的に検証できる体制を整備する。

### 背景

`potbot_lib` は ROS 非依存のコアライブラリであり、差動駆動ロボット運動学・ポテンシャルフィールド経路計画・PID 制御など、ナビゲーションシステムの根幹をなすアルゴリズムを実装している。ROS 1 (melodic) から ROS 2 (humble) への移行に伴い、ヘッダーファイルの `.h` → `.hpp` 変換や ROS 依存の除去といったリファクタリングが行われたため、各クラスが期待どおりに動作することを自動テストで継続的に確認できる仕組みが必要となった。

### 実装内容

- `potbot_lib/test/` ディレクトリにテストファイル 6 本を新規作成（合計 1418 行・110 テストケース）
- `potbot_lib/CMakeLists.txt` に `BUILD_TESTING=ON` 時の gtest 設定を追加
- テストフレームワーク: `ament_cmake_gtest` + Google Test

---

## 2. クラス継承階層

クラス階層の詳細は `M0_overview.md` の「クラス継承階層」セクションを参照。主な継承関係は以下の通り：

- `PID` および `OptimalPathFollower` は `DiffDriveAgent` を継承
- `ArtificialPotentialField` は `potential::Field` を継承

---

## 3. テストファイル構成

| ファイル | 行数 | テストケース数 | 対象クラス |
|---|---|---|---|
| `test/test_utility.cpp` | 257 行 | 27 ケース | `Point`, `Pose`, `utility` 関数群 |
| `test/test_diff_drive_agent.cpp` | 189 行 | 17 ケース | `DiffDriveAgent` |
| `test/test_pid.cpp` | 201 行 | 16 ケース | `PID` |
| `test/test_field.cpp` | 252 行 | 20 ケース | `Field` |
| `test/test_artificial_potential_field.cpp` | 307 行 | 19 ケース | `ArtificialPotentialField` |
| `test/test_apf_path_planner.cpp` | 212 行 | 11 ケース | `APFPathPlanner` |
| **合計** | **1418 行** | **110 ケース** | |

---

## 4. 各クラスのテスト仕様

### 4.1 `Point` / `Pose` 構造体 (test_utility.cpp)

#### テストスイート: `PointTest`

| テストケース名 | 検証内容 |
|---|---|
| `DefaultConstructor` | デフォルトコンストラクタで x=y=z=0.0 になること |
| `ParameterizedConstructor` | 引数付きコンストラクタで各値が正しく設定されること |
| `EigenVector3dConstructor` | `Eigen::Vector3d` からの変換コンストラクタが正しく動作すること |
| `Addition` | `operator+` で各成分が加算されること |
| `Subtraction` | `operator-` で各成分が減算されること |
| `ScalarMultiplication` | `operator*` でスカラー倍が正しく計算されること |
| `ScalarDivision` | `operator/` でスカラー除算が正しく計算されること |
| `Norm` | 3-4-0 ベクトルのノルムが 5.0 になること |
| `NormZeroVector` | ゼロベクトルのノルムが 0.0 になること |
| `Norm3D` | 3D ベクトルのノルムが `sqrt(14)` になること |
| `EqualityOperator` | `operator==` が同値・異値で正しく判定されること |
| `InequalityOperator` | `operator!=` が正しく判定されること |
| `ToTranslation` | `to_translation()` が `Eigen::Vector3d` に正しく変換されること |

#### テストスイート: `PoseTest`

| テストケース名 | 検証内容 |
|---|---|
| `DefaultConstructor` | デフォルトコンストラクタで position・rotation が全 0.0 になること |
| `ParameterizedConstructor` | 引数付きコンストラクタで position/rotation が正しく設定されること |
| `Addition` | `operator+` で position・rotation が加算されること |
| `Subtraction` | `operator-` で position・rotation が減算されること |
| `ScalarMultiplication` | `operator*` でスカラー倍が正しく計算されること |
| `EqualityOperator` | `operator==` が同値・異値で正しく判定されること |

#### テストスイート: `UtilityTest`

| テストケース名 | 検証内容 |
|---|---|
| `CombinationBasic` | `utility::combination(4, 2)` が 6.0 になること |
| `CombinationZero` | `utility::combination(5, 0)` が 1.0 になること |
| `CombinationN` | `utility::combination(5, 5)` が 1.0 になること |
| `GetRotateMatrix` | 90度回転行列が `[[0,-1],[1,0]]` になること |
| `GetRotateMatrixZero` | 0度回転行列が単位行列になること |
| `ContainsVector` | `utility::contains` が `std::vector` に対して正しく動作すること |
| `ContainsMap` | `utility::contains` が `std::map` に対して正しく動作すること |
| `FindClosestVector` | `utility::find_closest_vector` が最も近いベクトルを返すこと |

---

### 4.2 `DiffDriveAgent` (test_diff_drive_agent.cpp)

`DiffDriveAgent` は差動駆動ロボットの運動モデルを実装する基底クラスである。`PID` および `OptimalPathFollower` がこのクラスを継承する。

#### テストスイート: `DiffDriveAgentTest`

| テストケース名 | 検証内容 |
|---|---|
| `DefaultConstructor` | デフォルトコンストラクタで x=y=yaw=v=omega=0.0、deltatime=0.02 になること |
| `ParameterizedConstructor` | 引数付きコンストラクタで各パラメータが正しく設定されること |
| `UpdateStraightForward` | yaw=0 で直進したとき x 方向に移動すること |
| `UpdateRotation` | omega のみで旋回したとき yaw が変化し x/y は変化しないこと |
| `UpdateFacing90Degrees` | yaw=π/2 向きで直進したとき y 方向に移動すること |
| `UpdateOmegaBeforePosition` | `update()` が yaw を先に更新してから x/y を計算すること（更新順序の検証） |
| `UpdateMultipleSteps` | 10ステップ更新後に x=1.0 に到達すること |
| `GetDistanceToPoint` | `Point(3,4,0)` までの距離が 5.0 になること |
| `GetDistanceToSamePosition` | 同一座標への距離が 0.0 になること |
| `GetDistanceToPose` | `Pose(3,4,0)` までの距離が 5.0 になること |
| `GetDistanceNegativeCoords` | 負座標への距離が正しく計算されること |
| `GetAngleRight` | `+x` 方向への角度が 0.0 になること |
| `GetAngleUp` | `+y` 方向への角度が π/2 になること |
| `GetAngleLeft` | `-x` 方向への角度が π になること |
| `GetAngleDown` | `-y` 方向への角度が -π/2 になること |
| `GetAngleToPose` | `Pose(1,1,0)` への角度が π/4 になること |
| `GetAngleOffset` | ロボット位置がオフセットされた場合も角度が正しく計算されること |

---

### 4.3 `PID` (test_pid.cpp)（`DiffDriveAgent` を継承）

#### テストスイート: `PIDTest`

| テストケース名 | 検証内容 |
|---|---|
| `InitialProcessIsStop` | 初期状態でプロセスが `PROCESS_STOP` になること |
| `InitialVelocitiesAreZero` | 初期状態で v=omega=0.0 になること |
| `SetGain` | `setGain()` がコンパイル・実行できること |
| `SetMargin` | `setMargin()` がコンパイル・実行できること |
| `SetLimit` | `setLimit()` がコンパイル・実行できること |
| `ApplyLimitLinearVelocity` | v が上限を超えた場合にクランプされること |
| `ApplyLimitLinearVelocityNegative` | v が下限を下回った場合にクランプされること |
| `ApplyLimitAngularVelocity` | omega が上限を超えた場合にクランプされること |
| `ApplyLimitAngularVelocityNegative` | omega が下限を下回った場合にクランプされること |
| `ApplyLimitWithinBounds` | 上下限内の値はそのまま保持されること |
| `ReachedTargetAtOrigin` | ロボットと目標が同一位置の場合に `reachedTarget()` が true になること |
| `NotReachedTargetFarAway` | 目標が遠い場合に `reachedTarget()` が false になること |
| `ProcessTransitionFromStop` | `calculateCommand()` 呼び出しで `PROCESS_STOP` から遷移すること |
| `InitPIDResetsProcess` | `initPID()` でプロセス・速度が初期化されること |
| `CalculateCommandOutputsVelocity` | `calculateCommand()` 実行後に速度コマンドが出力されること |
| `ConvergesToTarget` | 複数ステップ更新後に目標に収束すること（統合テスト） |

---

### 4.4 `Field` (test_field.cpp)

`Field` はポテンシャルフィールドのグリッド管理を行う基底クラスである。`ArtificialPotentialField` がこのクラスを継承する。

#### テストスイート: `FieldTest`

| テストケース名 | 検証内容 |
|---|---|
| `DefaultConstructor` | コンストラクタでグリッドサイズ・解像度が正しく設定されること |
| `GridCount` | 4×5 グリッドの総セル数が 20 になること |
| `GridIndexing` | 各セルの row/col インデックスが正しく割り当てられること |
| `GridCoordinates` | 各セルの x/y 座標がグリッド位置と解像度から正しく計算されること |
| `GetFieldIndexCenter` | (row=1, col=1) のインデックスが 4 になること |
| `GetFieldIndexByCoordinate` | 座標 (0,0) のインデックスが 12 になること（5×5 グリッド） |
| `GetFieldIndexOutOfRangeThrows` | フィールド外座標で `std::out_of_range` 例外が投げられること |
| `CheckIndexOutOfRangeThrows` | 範囲外インデックスで `std::out_of_range` 例外が投げられること |
| `SetAndSearchFieldInfo` | `setFieldInfo()` と `searchFieldInfo()` が正しく連動すること |
| `SetFieldInfoFalse` | フラグを false に設定すると検索結果から除外されること |
| `SearchFieldInfoMultipleMatches` | 複数セルへのフラグ設定と検索が正しく動作すること |
| `SearchFieldInfoAndMode` | AND モードで複数フラグを持つセルのみが返されること |
| `SearchFieldInfoOrMode` | OR モードでいずれかのフラグを持つセルが返されること |
| `GetSquareIndexCenter` | 中心セルの周囲 8 セルが返されること |
| `GetSquareIndexCorner` | コーナーセルで境界外チェックにより空配列が返されること |
| `GetValueByIndex` | インデックス指定で正しい `FieldGrid` が返されること |
| `GetValueByCoordinate` | 座標指定で正しい `FieldGrid` が返されること |
| `InfoFilter` | `infoFilter()` でフラグ一致セルのみが抽出されること |
| `HeaderBounds` | `FieldHeader` の width/height/x_min/x_max/y_min/y_max が正しく計算されること |
| `HeaderWithNonZeroOrigin` | 原点オフセットありのグリッドで境界値が正しく計算されること |


---

### 4.5 `ArtificialPotentialField` (test_artificial_potential_field.cpp)（`Field` を継承）

#### テストスイート: `APFTest`

| テストケース名 | 検証内容 |
|---|---|
| `DefaultConstructor` | 5×5 グリッドが 25 セルで初期化されること |
| `InitPotentialField` | `initPotentialField()` で 7×7 グリッドが 49 セルで初期化されること |
| `SetGoalByCoordinate` | `setGoal()` でゴール座標が正しく保存されること |
| `SetGoalMarksFieldGrid` | `setGoal()` で対応するグリッドに `IS_GOAL` フラグが立つこと |
| `SetGoalMarksAroundGoal` | `setGoal()` でゴール周囲に `IS_AROUND_GOAL` フラグが立つこと |
| `SetRobotByCoordinate` | `setRobot()` でロボット座標が正しく保存されること |
| `SetRobotMarksFieldGrid` | `setRobot()` で対応するグリッドに `IS_ROBOT` フラグが立つこと |
| `SetObstacleByCoordinate` | `setObstacle()` で障害物座標が正しく登録されること |
| `SetMultipleObstacles` | 複数の障害物が正しく登録されること |
| `ClearObstacles` | `clearObstacles()` で障害物リストが空になること |
| `SetObstacleByEigenVector` | `Eigen::Vector2d` 経由でも障害物を設定できること |
| `AttractionIncreaseWithDistanceToGoal` | ゴールから遠いほど吸引ポテンシャルが大きくなること |
| `RepulsionIncreaseNearObstacle` | 障害物に近いほど斥力ポテンシャルが大きくなること、閾値外は 0 になること |
| `TotalPotentialIsAttractionPlusRepulsion` | 各セルの `potential = attraction + repulsion` が成立すること |
| `PotentialMinimumNearGoal` | ゴール付近に合計ポテンシャルの最小値が存在すること |
| `LocalMinimumDetection` | 障害物が存在する場合も `createPotentialField()` がクラッシュしないこと |
| `GetAttractionFieldUpdatesValues` | `getAttractionField()` 後に `value` フィールドが `attraction` と一致すること |
| `GetRepulsionFieldUpdatesValues` | `getRepulsionField()` 後に `value` フィールドが `repulsion` と一致すること |
| `SetParams` | `setParams()` で `distance_threshold_repulsion_field` が更新されること |

---

### 4.6 `APFPathPlanner` (test_apf_path_planner.cpp)

#### テストスイート: `APFPathPlannerTest`

| テストケース名 | 検証内容 |
|---|---|
| `Constructor` | `ArtificialPotentialField` ポインタを渡してコンストラクトできること |
| `SetParams` | `setParams()` がコンパイル・実行できること |
| `CreatePathReturnsTrue` | 障害物なし・単純なシナリオで `createPath()` が true を返すこと |
| `CreatePathProducesNonEmptyPath` | `createPath()` 後に `getPath()` で空でない経路が得られること |
| `CreatePathStartsAtRobot` | 経路の始点がロボット位置の 1 グリッド以内にあること |
| `CreatePathEndsNearGoal` | 高解像度グリッド（res=0.05m）でゴール付近で経路が終端すること |
| `CreatePathWithObstacle` | 障害物がある場合も `createPath()` が true を返し空でない経路が得られること |
| `CreatePathWithWeightReturnsTrue` | `createPathWithWeight()` が true を返すこと |
| `CreatePathWithWeightProducesPath` | `createPathWithWeight()` 後に空でない経路が得られること |
| `BezierOnEmptyPathReturnsFalse` | 経路なし状態で `bezier()` が false を返すこと |
| `BezierAfterCreatePath` | `createPath()` 後に `bezier()` が true を返しスムーズ経路が得られること |

---

## 5. テストコマンド

### ビルド（テスト有効化）

```bash
cd /home/rtx3090/potbot/ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON
```

### 全テスト実行

```bash
ctest --test-dir build/potbot_lib
```

### 個別テスト実行

```bash
ctest --test-dir build/potbot_lib -R test_utility
ctest --test-dir build/potbot_lib -R test_diff_drive_agent
ctest --test-dir build/potbot_lib -R test_pid
ctest --test-dir build/potbot_lib -R test_field
ctest --test-dir build/potbot_lib -R test_artificial_potential_field
ctest --test-dir build/potbot_lib -R test_apf_path_planner
```

### 詳細出力付き実行

```bash
ctest --test-dir build/potbot_lib --output-on-failure
```

---

## 6. テストカバレッジ状況

### テスト済みクラス

| クラス | テストスイート | テストケース数 |
|---|---|---|
| `Point` | `PointTest` | 13 |
| `Pose` | `PoseTest` | 6 |
| `utility` 関数群 | `UtilityTest` | 8 |
| `DiffDriveAgent` | `DiffDriveAgentTest` | 17 |
| `PID` | `PIDTest` | 16 |
| `Field` | `FieldTest` | 20 |
| `ArtificialPotentialField` | `APFTest` | 19 |
| `APFPathPlanner` | `APFPathPlannerTest` | 11 |

### 未テストクラス

| クラス | 実装ファイル | 行数目安 | 備考 |
|---|---|---|---|
| `OptimalPathFollower` | `src/optimal_path_follower.cpp` | 327 行 | 経路追従制御器（`DiffDriveAgent` を継承） |
| `Interpolate` | `src/interpolate.cpp` | 150 行以上 | Bezier 曲線ユーティリティ |

> `Interpolate` については `APFPathPlanner` の `bezier()` テストを通じて間接的に検証されているが、直接テストは行われていない。

---

## 7. 使用したテストパターン

### アサーションマクロ

| マクロ | 用途 | 使用例 |
|---|---|---|
| `EXPECT_DOUBLE_EQ(a, b)` | 浮動小数点の厳密一致 | コンストラクタ初期値の検証 |
| `EXPECT_NEAR(a, b, tol)` | 浮動小数点の近似一致 | 三角関数・行列演算の結果検証（許容誤差 `1e-9`） |
| `EXPECT_EQ(a, b)` | 整数・符号なし整数の一致 | グリッドインデックス・サイズの検証 |
| `EXPECT_GT(a, b)` | `a > b` の確認 | 経路サイズが 0 より大きいこと |
| `EXPECT_LT(a, b)` | `a < b` の確認 | ポテンシャル値の大小比較 |
| `EXPECT_GE(a, b)` | `a >= b` の確認 | フラグセット後の検索結果サイズ |
| `EXPECT_TRUE(cond)` | 条件が真であること | `reachedTarget()`、`createPath()` の戻り値 |
| `EXPECT_FALSE(cond)` | 条件が偽であること | `reachedTarget()` 未到達時 |
| `EXPECT_THROW(expr, type)` | 特定例外が発生すること | 範囲外インデックス・座標アクセス |
| `ASSERT_EQ(a, b)` | 一致しない場合にテストを中断 | コンテナサイズの前提条件確認 |
| `ASSERT_GT(a, b)` | 不等条件でテストを中断 | 経路サイズの前提条件確認 |
| `SUCCEED()` | 無条件成功（コンパイル確認用） | `setGain()`、`setMargin()` 等の API 存在確認 |

### テストパターン

- **デフォルト状態の検証**: 全クラスのコンストラクタで初期値を検証
- **境界値テスト**: フィールド外座標アクセスで例外発生を検証
- **物理モデルの整合性テスト**: ポテンシャル値の大小関係をアルゴリズム特性から検証
- **統合テスト（シミュレーション）**: `PID::ConvergesToTarget` で最大 1000 ステップの収束を確認
- **API 存在確認テスト**: `SUCCEED()` を用いてコンパイル・API インターフェースの確認のみを行うケース

---

## 8. CMakeLists.txt テスト設定

`potbot_lib/CMakeLists.txt` に以下の設定が追加された：

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_LIBS
    potbot_lib_utility
  )

  ament_add_gtest(test_utility test/test_utility.cpp)
  target_include_directories(test_utility PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_utility ${TEST_LIBS})

  # (同様に test_diff_drive_agent, test_pid, test_field,
  #  test_artificial_potential_field, test_apf_path_planner も登録)
endif()
```

各テスト実行形式は `ament_add_gtest` マクロで登録され、`potbot_lib_utility` ライブラリとリンクされる。

---

## 9. 今後の課題

### 未テストクラスへの対応

1. **`OptimalPathFollower` のユニットテスト追加**（`DiffDriveAgent` を継承）
   - 経路追従制御器であり、速度コマンド出力・目標収束・制限適用を検証する必要がある
   - `DiffDriveAgent` を継承しているため、親クラスの運動モデルを活用した統合テストも検討

2. **`Interpolate` のユニットテスト追加**
   - Bezier 曲線補間の数値精度を独立したテストで検証
   - `APFPathPlanner::bezier()` テストでの間接検証を直接テストに昇格させる

### テストカバレッジの向上

- 境界値テストの拡充（フィールドエッジ付近の挙動）
- ローカルミニマム回避アルゴリズムの動作検証
- 異常系（不正パラメータ・空入力）への対応テスト追加

---

*作成日: 2026-03-21*
*マイルストーン: M0*
*チケット: T-001*
