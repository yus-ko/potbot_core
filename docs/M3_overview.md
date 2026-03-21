# M3 マイルストーン概要仕様書 — potbot_plugin テスト強化

| 項目 | 内容 |
|---|---|
| マイルストーン | M3 |
| タイトル | potbot_plugin テスト強化 |
| ステータス | 計画中 |
| 予定開始日 | 未定 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M3 は `potbot_plugin` パッケージのテスト品質を M2 で `potbot_lib` に適用したレベルまで引き上げることを目的とするマイルストーンである。

具体的には以下の3点を達成する予定である：

1. **APFプランナーのパラメータ化テスト追加**（T-019）: `potbot_plugin/test/test_apf_planner.cpp` に対して、現在欠落しているパラメータ化テストを追加し、複数のスタート・ゴール座標および障害物配置に対する動作を網羅的に検証する。
2. **OptimalPathFollowerのパラメータ化テスト追加**（T-020）: `potbot_plugin/test/test_optimal_path_follower_plugin.cpp` に速度制限・インターフェースの境界値テストを追加し、Nav2 Controller プラグインとしての仕様適合性を強化する。
3. **linting 完全対応**（T-021）: cpplint / uncrustify による静的解析を `potbot_plugin` の全テストファイルに適用し、コードスタイルを統一する。

---

## 2. 背景

### M1 で作成されたテストの現状

M1 では `potbot_plugin` の2つのプラグインクラスに対してそれぞれユニットテストを新規作成した。しかし、その内容は以下の最小限の検証にとどまっている：

**`test_apf_planner.cpp`（現在4件）:**

| テスト名 | 検証内容 |
|---|---|
| `InstanceIsValid` | デフォルト構築が成功し `nullptr` でないこと |
| `InheritsGlobalPlannerInterface` | `nav2_core::GlobalPlanner` を継承していること |
| `PlanReturnTypeIsPath` | `createPlan()` の戻り値型が `nav_msgs::msg::Path` であること |
| `MultipleInstancesNoInterference` | 複数インスタンスの同時生成が成功すること |

**`test_optimal_path_follower_plugin.cpp`（現在6件）:**

| テスト名 | 検証内容 |
|---|---|
| `ConstructorNoThrow` | デフォルトコンストラクタが例外なく成功すること |
| `InstanceIsValid` | `nullptr` でないこと |
| `InheritsControllerInterface` | `nav2_core::Controller` を継承していること |
| `SetSpeedLimitAbsolute` | `setSpeedLimit()` 絶対値呼び出しが安全なこと |
| `SetSpeedLimitPercentage` | `setSpeedLimit()` パーセント呼び出しが安全なこと |
| `MultipleInstancesCanBeCreated` | 複数インスタンスの同時生成が成功すること |

### M2 との比較で明らかになったギャップ

M2 では `potbot_lib` に対して以下のパラメータ化テストを追加し、アルゴリズムの動作をより広い入力範囲で検証できるようにした：

- `APFGainTest`: 引力・斥力ゲイン4パターン × 3テストケース
- `APFAttrGainScaleTest`: 引力ゲインスケール比較3パターン
- `APFDtrTest`: 障害物距離閾値（dtr）3パターン × 2テストケース
- `APFGoalTest`: ゴール位置5パターン × 2テストケース
- `APFMultiObstacleTest`: 複数障害物配置3パターン × 2テストケース

一方、`potbot_plugin` には同等のパラメータ化テストが一切存在しない。Nav2 プラグインとして実際の経路計画・制御を担う層でのパラメータ耐性検証が不足している状態である。

---

## 3. スコープ（予定）

### 対象パッケージ

| パッケージ | 対象 | 理由 |
|---|---|---|
| `potbot_plugin` | **対象** | M1 テストのパラメータ化が未実施 |
| `potbot_lib` | 対象外 | M2 で対応済み |
| `potbot_ros` | 対象外 | M3 スコープ外 |
| `potbot_behavior_tree` | 対象外 | M4 で対応予定 |

### 追加予定テスト（T-019）— APFプランナー

Nav2 の `Costmap2DROS` 依存により `configure()` を伴う結合テストは困難であるが、以下のパラメータ化テストを追加する予定である：

| テストスイート | パラメータ | 検証内容 |
|---|---|---|
| `APFPlannerInterfaceParamTest` | プランナーインスタンス数 (1, 3, 5) | 複数インスタンスの同時生成が常に安全なこと |
| `APFPlannerAllocParamTest` | メモリ確保パターン（unique_ptr / shared_ptr / スタック割り当て） | いずれの確保方法でも有効なインスタンスが得られること（別テストケースとして実装） |
| `APFPlannerCastParamTest` | 異なる基底クラスへのキャスト先 | `GlobalPlanner` インターフェースへのキャストが成功すること |

### 追加予定テスト（T-020）— OptimalPathFollower

| テストスイート | パラメータ | 検証内容 |
|---|---|---|
| `SpeedLimitAbsoluteParamTest` | 速度値 (0.0, 0.1, 0.5, 1.0, 2.0) | 任意の絶対速度制限値で `setSpeedLimit()` がクラッシュしないこと |
| `SpeedLimitPercentageParamTest` | パーセント値 (0.0, 25.0, 50.0, 100.0, 200.0) | 任意のパーセント値で `setSpeedLimit()` がクラッシュしないこと |
| `ControllerCastParamTest` | キャスト先クラス | `nav2_core::Controller` へのキャストが成功すること |

### linting（T-021）

- `potbot_plugin/test/` 以下の全 `.cpp` ファイルに cpplint を適用
- `CMakeLists.txt` に `ament_lint_auto_find_test_dependencies()` を確認・追加
- 行末スペース・不要インクルードガード等を uncrustify で自動修正

---

## 4. チケット一覧（予定）

### T-019: APFプランナー パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-019 |
| タイトル | potbot_plugin の APFプランナーにパラメータ化テストを追加 |
| ステータス | 未着手 |

**予定実装内容:**

- `potbot_plugin/test/test_apf_planner.cpp` を拡張し、パラメータ化テストスイートを追加
- `::testing::TestWithParam<T>` パターンを使用し、`INSTANTIATE_TEST_SUITE_P` でテストケースを定義
- `configure()` を呼ばなくても検証できるインターフェース仕様を対象とする

**対応ファイル（予定）:**

- `potbot_plugin/test/test_apf_planner.cpp`

---

### T-020: OptimalPathFollower パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-020 |
| タイトル | potbot_plugin の OptimalPathFollower にパラメータ化テストを追加 |
| ステータス | 未着手 |

**予定実装内容:**

- `potbot_plugin/test/test_optimal_path_follower_plugin.cpp` を拡張
- 速度制限値（絶対値・パーセント両方）の境界値をパラメータ化
- `setSpeedLimit()` が任意の非負浮動小数点値に対して安全であることを多パターンで検証

**対応ファイル（予定）:**

- `potbot_plugin/test/test_optimal_path_follower_plugin.cpp`

---

### T-021: potbot_plugin テストファイルの linting 対応

| 項目 | 内容 |
|---|---|
| チケット番号 | T-021 |
| タイトル | potbot_plugin テストファイルに cpplint/uncrustify を適用 |
| ステータス | 未着手 |

**予定実装内容:**

- `ament_cpplint` / `ament_uncrustify` によるスタイルチェックを追加
- `potbot_plugin/CMakeLists.txt` の `BUILD_TESTING` ブロックにリント設定を追加
- 既存テストファイルのスタイル違反を修正

**対応ファイル（予定）:**

- `potbot_plugin/CMakeLists.txt`
- `potbot_plugin/test/test_apf_planner.cpp`
- `potbot_plugin/test/test_optimal_path_follower_plugin.cpp`

---

## 5. 完了基準

M3 を完了とみなすための基準は以下の通りである。

| # | 完了基準 |
|---|---|
| 1 | `potbot_plugin/test/test_apf_planner.cpp` に `INSTANTIATE_TEST_SUITE_P` を使用したパラメータ化テストが3スイート以上追加されていること |
| 2 | `potbot_plugin/test/test_optimal_path_follower_plugin.cpp` に速度制限値パラメータ化テストが2スイート以上追加されていること |
| 3 | `colcon build --packages-select potbot_plugin --cmake-args -DBUILD_TESTING=ON` でビルドが成功すること |
| 4 | `ctest --test-dir build/potbot_plugin` で全テストが合格すること |
| 5 | `ament_cpplint` によるスタイルチェックが全テストファイルでエラーなく通ること |
| 6 | テスト総数が T-019・T-020 追加分を含めて 20件以上になること |

---

## 6. 関連ファイル一覧（予定）

### 既存テスト（M1 作成・M3 で拡張予定）

```
potbot_plugin/
├── CMakeLists.txt                              # linting 設定追加予定
└── test/
    ├── test_apf_planner.cpp                    # パラメータ化テスト追加予定（T-019）
    └── test_optimal_path_follower_plugin.cpp   # パラメータ化テスト追加予定（T-020）
```

### 参照ファイル（実装パターン参考）

```
potbot_lib/
└── test/
    └── test_apf_param.cpp                      # パラメータ化テストの実装パターン参考
```

### 本仕様書

```
docs/
└── M3_overview.md                              # 本ファイル（M3マイルストーン概要仕様書）
```
