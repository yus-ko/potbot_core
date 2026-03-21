# M2 マイルストーン概要仕様書 — パラメータ化テスト・統合テスト・リグレッションテスト

| 項目 | 内容 |
|---|---|
| マイルストーン | M2 |
| タイトル | パラメータ化テスト・統合テスト・リグレッションテスト |
| ステータス | 完了 |
| 完了日 | 2026-03-21 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M2 は M1 で整備したユニットテスト基盤をさらに発展させ、テストの網羅性・堅牢性・再発防止能力を高めることを目的としたマイルストーンである。

具体的には以下の3点を達成する：

1. **パラメータ化テスト（T-013〜T-016）**: Google Test の `TEST_P` マクロを用いた多値テストを導入し、各クラスの挙動を複数のパラメータ組み合わせで体系的に検証する。
2. **統合テスト（T-017）**: APFPathPlanner + OptimalPathFollower + DiffDriveAgent の3コンポーネントを連携させ、ナビゲーションパイプライン全体の動作を end-to-end で検証する。
3. **リグレッションテスト（T-018）**: M0・M1 で発見・対処した既知エッジケースをテストとして記録し、将来的なリグレッションを防止する。

---

## 2. 背景

### M0・M1 との関係

M0 では `potbot_lib` の主要クラスに対して基本的なユニットテストを整備し、M1 ではテストカバレッジの拡充と CI 環境の整備を行った。M2 では以下の観点からテスト品質をさらに向上させる必要があった：

- **パラメータ網羅性の不足**: M0 のテストは代表的な1〜2値のみを検証しており、境界値や複数パラメータの組み合わせがカバーされていなかった。
- **コンポーネント間の連携未検証**: 各クラスは単体でテストされていたが、実際のナビゲーション実行時における APF → PathPlanner → Controller → Agent の一連の動作は検証されていなかった。
- **既知エッジケースの再発リスク**: 開発中に発見されたエッジケース（ゼロ距離、空経路、数値オーバーフロー等）が後のリファクタリングで再発するリスクがあった。

### TEST_P を採用した理由

Navigation 2 プロジェクト（nav2_controller, nav2_costmap_2d 等）のテストスタイルを参考に、`TEST_P` + `INSTANTIATE_TEST_SUITE_P` の組み合わせを採用した。これにより：

- 同一のテストロジックを複数のパラメータセットに適用でき、コードの重複を排除できる
- テスト失敗時にどのパラメータセットで失敗したかが明確になる
- パラメータ追加のみで新たなケースを追加できる

---

## 3. スコープ

### 対象パッケージ

| パッケージ | 対象 | 理由 |
|---|---|---|
| `potbot_lib` | **対象** | ROS 非依存のコアアルゴリズム。外部依存なしでテスト可能 |
| `potbot_ros` | 対象外 | ROS 2 ランタイムが必要 |
| `potbot_plugin` | 対象外 | Nav2 フレームワークへの依存がある |
| `potbot_behavior_tree` | 対象外 | 同上 |
| `potbot_msgs` | 対象外 | メッセージ定義のみ |

### テスト対象クラス

| クラス | 追加テストファイル | テスト件数 |
|---|---|---|
| `DiffDriveAgent` | `test_diff_drive_agent_param.cpp` | 16件 |
| `PID` | `test_pid_param.cpp` | 21件 |
| `Field` | `test_field_param.cpp` | 50件 |
| `ArtificialPotentialField` | `test_apf_param.cpp` | 37件 |
| `ApfPathPlanner` | `test_apf_path_planner_param.cpp` | 27件 |
| `OptimalPathFollower` | `test_optimal_path_follower_param.cpp` | 24件 |
| `Interpolate` | `test_interpolate_param.cpp` | 68件 |
| APF パイプライン（統合） | `integration/test_apf_pipeline.cpp` | 3件 |
| 既知エッジケース（リグレッション） | `regression/test_regression.cpp` | 15件 |
| **合計** | 9ファイル | **261件** |

---

## 4. チケット一覧

### T-013: DiffDriveAgent・PID の TEST_P パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-013 |
| タイトル | DiffDriveAgent・PID の TEST_P パラメータ化テストを追加 |
| コミット | `f248d19` |
| ステータス | 完了 |

---

### T-014: Field・ArtificialPotentialField の TEST_P パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-014 |
| タイトル | Field・ArtificialPotentialField の TEST_P パラメータ化テストを追加 |
| コミット | `b877741` |
| ステータス | 完了 |

---

### T-015: ApfPathPlanner の TEST_P パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-015 |
| タイトル | ApfPathPlanner の TEST_P パラメータ化テストを追加 |
| コミット | `9d9c9d4` |
| ステータス | 完了 |

---

### T-016: OptimalPathFollower・Interpolate の TEST_P パラメータ化テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-016 |
| タイトル | OptimalPathFollower・Interpolate の TEST_P パラメータ化テストを追加 |
| コミット | `372aba6` |
| ステータス | 完了 |

---

### T-017: APF パイプライン統合テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-017 |
| タイトル | APF パイプライン統合テストを追加（APFPathPlanner + OptimalPathFollower + DiffDriveAgent 連携） |
| コミット | `832fad4` |
| ステータス | 完了 |

---

### T-018: リグレッションテスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-018 |
| タイトル | リグレッションテストを追加（既知エッジケースの再発防止） |
| コミット | `514a63b` |
| ステータス | 完了 |

---

## 5. 完了基準

M2 を完了とみなすための基準は以下の通りである。すべての項目が達成済みであることを確認した。

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | `DiffDriveAgent`・`PID` の TEST_P パラメータ化テストが作成されていること | 完了 |
| 2 | `Field`・`ArtificialPotentialField` の TEST_P パラメータ化テストが作成されていること | 完了 |
| 3 | `ApfPathPlanner` の TEST_P パラメータ化テストが作成されていること | 完了 |
| 4 | `OptimalPathFollower`・`Interpolate` の TEST_P パラメータ化テストが作成されていること | 完了 |
| 5 | APF パイプライン統合テストが `integration/` ディレクトリに作成されていること | 完了 |
| 6 | リグレッションテストが `regression/` ディレクトリに作成されていること | 完了 |
| 7 | `colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON` でビルドが成功すること | 完了 |
| 8 | `ctest --test-dir build/potbot_lib` で全テスト（M0〜M2 追加分）が合格すること | 完了 |

---

## 6. 関連ファイル一覧

### パラメータ化テスト（T-013〜T-016）

```
potbot_lib/
└── test/
    ├── test_diff_drive_agent_param.cpp   # DiffDriveAgent TEST_P テスト (16件)
    ├── test_pid_param.cpp                # PID TEST_P テスト (21件)
    ├── test_field_param.cpp              # Field TEST_P テスト (50件)
    ├── test_apf_param.cpp                # ArtificialPotentialField TEST_P テスト (37件)
    ├── test_apf_path_planner_param.cpp   # ApfPathPlanner TEST_P テスト (27件)
    ├── test_optimal_path_follower_param.cpp  # OptimalPathFollower TEST_P テスト (24件)
    └── test_interpolate_param.cpp        # Interpolate TEST_P テスト (68件)
```

### 統合テスト（T-017）

```
potbot_lib/
└── test/
    └── integration/
        ├── CMakeLists.txt                # 統合テスト用ビルド設定
        └── test_apf_pipeline.cpp         # APFパイプライン統合テスト (3件)
```

### リグレッションテスト（T-018）

```
potbot_lib/
└── test/
    └── regression/
        ├── CMakeLists.txt                # リグレッションテスト用ビルド設定
        └── test_regression.cpp           # 既知エッジケースリグレッションテスト (15件)
```

### 本仕様書

```
docs/
├── M2_overview.md                              # 本ファイル（M2マイルストーン概要仕様書）
├── M2_T013_parameterized_diff_drive_pid.md     # T-013 チケット仕様書
├── M2_T014_parameterized_field_apf.md          # T-014 チケット仕様書
├── M2_T015_parameterized_apf_path_planner.md   # T-015 チケット仕様書
├── M2_T016_parameterized_optimal_path_follower.md  # T-016 チケット仕様書
├── M2_T017_integration_test_apf_pipeline.md    # T-017 チケット仕様書
└── M2_T018_regression_tests.md                 # T-018 チケット仕様書
```
