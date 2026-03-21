# M4 マイルストーン概要仕様書 — potbot_behavior_tree テスト + CI整備

| 項目 | 内容 |
|---|---|
| マイルストーン | M4 |
| タイトル | potbot_behavior_tree テスト + CI整備 |
| ステータス | 計画中 |
| 予定開始日 | 未定 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M4 は `potbot_behavior_tree` パッケージに初のユニットテストを導入し、あわせて GitHub Actions による CI パイプラインを整備することを目的とするマイルストーンである。

具体的には以下の3点を達成する予定である：

1. **RotateToGoalDirection アクションサーバーのユニットテスト作成**（T-022）: `potbot_nav::behavior::RotateToGoalDirection` クラスに対する gtest ベースのユニットテストを新規作成し、アクションサーバーとして期待されるインターフェースを検証する。
2. **WaitAction・SpinAction BTノードのユニットテスト作成**（T-023）: `potbot_behavior_tree::WaitAction` および `potbot_nav::SpinAction` BTノードに対するユニットテストを作成し、ポート定義・継承関係・インスタンス生成の安全性を検証する。
3. **GitHub Actions CI パイプラインの整備**（T-024）: push / pull request 時に `potbot_lib`・`potbot_plugin`・`potbot_behavior_tree` の全テストを自動実行する CI ワークフローを構築し、手動 Docker 実行への依存を解消する。

---

## 2. 背景

### potbot_behavior_tree のテスト未整備状態

M0〜M5 を通じて、`potbot_behavior_tree` には一切のテストが作成されていない。現在のディレクトリ構成に `test/` ディレクトリが存在せず、`CMakeLists.txt` にも `BUILD_TESTING` ブロックがない。

この状態は以下のリスクを生じさせる：

- `RotateToGoalDirection` の PID 制御パラメータ（`gain_p_`, `gain_i_`, `gain_d_`, `tolerance_angle_`）が仕様通りに設定・動作するかを継続的に検証する手段がない
- `WaitAction` の `wait_duration` ポートや `SpinAction` の PID ゲインポートの型定義が変更された際に気づく仕組みがない
- `BtActionNode` 継承クラスのコンストラクタシグネチャが Nav2 アップデートにより変化した場合でも CI で検出できない

### CI パイプラインの不在

現状、テスト実行は M0 で構築した Docker 環境（`test/docker-compose.yml`）を手動で起動することに依存している。GitHub へのプッシュ時に自動テストが実行されないため、レグレッションの検出が遅れるリスクがある。

M4 で GitHub Actions CI を整備することで、すべての PR・プッシュに対して自動テストが実行される体制を構築する。

### テスト対象クラスの概要

**`potbot_nav::behavior::RotateToGoalDirection`**（`rotate_to_goal_direction.hpp`）:

- `nav2_behaviors::TimedBehavior<GoalPoseAction>` を継承するアクションサーバー
- `onRun()`: ゴール姿勢を受け取り制御を開始する
- `onConfigure()`: PID ゲインパラメータを初期化する
- `onCycleUpdate()`: PID 制御により目標方向への回転を継続する
- 内部状態: `gain_p_`, `gain_i_`, `gain_d_`, `tolerance_angle_`, `goal_pose_`, `controller_`（PID）

**`potbot_nav::SpinAction`**（`rotate_to_goal_direction_action.hpp`）:

- `nav2_behavior_tree::BtActionNode<potbot_msgs::action::GoalPose>` を継承する BT アクションノード
- `on_tick()`: BT ティック時にゴールをアクションサーバーへ送信する
- ポート: `goal`（PoseStamped）、`gain_p`（double, デフォルト1.0）、`gain_i`（double, デフォルト0.1）、`gain_d`（double, デフォルト0.01）

**`potbot_behavior_tree::WaitAction`**（`wait_action.hpp`）:

- `nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>` を継承する BT アクションノード
- `on_tick()`: BT ティック時に待機アクションをサーバーへ送信する
- ポート: `wait_duration`（int, デフォルト1、単位：秒）

---

## 3. スコープ（予定）

### 対象パッケージ

| パッケージ | 対象 | 理由 |
|---|---|---|
| `potbot_behavior_tree` | **対象** | テストが一切存在しない |
| `potbot_plugin` | 対象外 | M3 で対応予定 |
| `potbot_lib` | 対象外 | M2 で対応済み |
| `potbot_ros` | 対象外 | M4 スコープ外 |

### 追加予定テスト（T-022）— RotateToGoalDirection

Nav2 Lifecycle の依存により `configure()` を伴う結合テストは困難であるが、以下のユニットテストを追加する予定である：

| テスト名 | 検証内容 |
|---|---|
| `ConstructorNoThrow` | デフォルトコンストラクタが例外なく成功すること |
| `InstanceIsValid` | `make_unique<>` で生成したインスタンスが `nullptr` でないこと |
| `InheritsTimedBehaviorInterface` | `nav2_behaviors::TimedBehavior<GoalPoseAction>` を継承していること |
| `MultipleInstancesNoInterference` | 複数インスタンスの同時生成が安全なこと |

### 追加予定テスト（T-023）— WaitAction・SpinAction BTノード

BT ノードは `BehaviorTreeFactory` への登録なしに直接インスタンス化できないため、インターフェース検証を中心とする：

| テスト名 | 対象クラス | 検証内容 |
|---|---|---|
| `WaitAction_ProvidedPortsNotEmpty` | `WaitAction` | `providedPorts()` が空でないこと |
| `WaitAction_WaitDurationPortExists` | `WaitAction` | `wait_duration` ポートが定義されていること |
| `SpinAction_ProvidedPortsNotEmpty` | `SpinAction` | `providedPorts()` が空でないこと |
| `SpinAction_GainPortsExist` | `SpinAction` | `gain_p`, `gain_i`, `gain_d` ポートが定義されていること |
| `SpinAction_GoalPortExists` | `SpinAction` | `goal` ポートが定義されていること |

### CI パイプライン（T-024）

| 対象 | 内容 |
|---|---|
| トリガー | `push`（humble / main ブランチ）および `pull_request` |
| ビルド環境 | ROS 2 Humble on Ubuntu 22.04（Docker） |
| テスト対象 | `potbot_lib`, `potbot_plugin`, `potbot_behavior_tree` |
| 成果物 | テスト結果サマリー（GitHub Actions Summary） |

---

## 4. チケット一覧（予定）

### T-022: RotateToGoalDirection ユニットテスト新規作成

| 項目 | 内容 |
|---|---|
| チケット番号 | T-022 |
| タイトル | potbot_behavior_tree の RotateToGoalDirection ユニットテストを新規作成 |
| ステータス | 未着手 |

**予定実装内容:**

- `potbot_behavior_tree/test/test_rotate_to_goal_direction.cpp` を新規作成
- rclcpp の初期化・終了を `SetUpTestSuite` / `TearDownTestSuite` で管理
- `TimedBehavior` 継承の検証には `std::dynamic_pointer_cast` を使用
- `potbot_behavior_tree/CMakeLists.txt` に `BUILD_TESTING` ブロックを追加し、gtest 依存を設定

**対応ファイル（予定）:**

- `potbot_behavior_tree/test/test_rotate_to_goal_direction.cpp`（新規作成）
- `potbot_behavior_tree/CMakeLists.txt`（BUILD_TESTING 設定追加）

---

### T-023: WaitAction・SpinAction BTノード ユニットテスト新規作成

| 項目 | 内容 |
|---|---|
| チケット番号 | T-023 |
| タイトル | WaitAction と SpinAction のユニットテストを新規作成 |
| ステータス | 未着手 |

**予定実装内容:**

- `potbot_behavior_tree/test/test_bt_action_nodes.cpp` を新規作成
- `providedPorts()` はスタティックメソッドのため、インスタンス化なしに呼び出し可能
- `BT::PortsList` の内容（キー名・型）を検証することでポート定義の仕様適合性を保証
- テストは `SetUpTestSuite` / `TearDownTestSuite` で rclcpp を管理

**対応ファイル（予定）:**

- `potbot_behavior_tree/test/test_bt_action_nodes.cpp`（新規作成）

---

### T-024: GitHub Actions CI パイプライン整備

| 項目 | 内容 |
|---|---|
| チケット番号 | T-024 |
| タイトル | GitHub Actions による自動テスト CI パイプラインを整備 |
| ステータス | 未着手 |

**予定実装内容:**

- `.github/workflows/ci.yml` を新規作成
- `humble` / `main` ブランチへの push および pull_request をトリガーとする
- `ros-tooling/setup-ros@v0.7` で ROS 2 Humble 環境を構築
- `colcon build --cmake-args -DBUILD_TESTING=ON` でビルド
- `ctest` または `colcon test` でテストを実行し、結果を GitHub Actions Summary に表示
- テスト失敗時はワークフローを失敗ステータスで終了する

**対応ファイル（予定）:**

- `.github/workflows/ci.yml`（新規作成）

---

## 5. 完了基準

M4 を完了とみなすための基準は以下の通りである。

| # | 完了基準 |
|---|---|
| 1 | `potbot_behavior_tree/test/` ディレクトリが作成され、2ファイル以上のテストが存在すること |
| 2 | `colcon build --packages-select potbot_behavior_tree --cmake-args -DBUILD_TESTING=ON` でビルドが成功すること |
| 3 | `ctest --test-dir build/potbot_behavior_tree` で全テストが合格すること |
| 4 | `RotateToGoalDirection` に対して4件以上のユニットテストが作成されていること |
| 5 | `WaitAction` および `SpinAction` のポート定義検証テストが合計5件以上作成されていること |
| 6 | `.github/workflows/ci.yml` が作成され、`humble` ブランチへの push で CI が自動実行されること |
| 7 | CI ワークフロー内で `potbot_lib`・`potbot_plugin`・`potbot_behavior_tree` の全テストが実行されること |

---

## 6. 関連ファイル一覧（予定）

### 新規テストファイル（T-022・T-023）

```
potbot_behavior_tree/
├── CMakeLists.txt                                    # BUILD_TESTING 設定追加予定
└── test/                                             # 新規作成予定
    ├── test_rotate_to_goal_direction.cpp             # RotateToGoalDirection テスト（T-022）
    └── test_bt_action_nodes.cpp                      # WaitAction・SpinAction テスト（T-023）
```

### CI パイプライン（T-024）

```
.github/
└── workflows/
    └── ci.yml                                        # GitHub Actions CI ワークフロー（新規作成予定）
```

### テスト対象ヘッダー（参照のみ）

```
potbot_behavior_tree/
└── include/
    └── potbot_behavior_tree/
        └── plugins/
            ├── rotate_to_goal_direction.hpp          # RotateToGoalDirection クラス定義
            └── action/
                ├── rotate_to_goal_direction_action.hpp  # SpinAction（BT ノード）クラス定義
                └── wait_action.hpp                   # WaitAction クラス定義
```

### 参照ファイル（実装パターン参考）

```
potbot_plugin/
└── test/
    ├── test_apf_planner.cpp                          # Nav2 プラグインテストのパターン参考
    └── test_optimal_path_follower_plugin.cpp         # Nav2 プラグインテストのパターン参考
```

### 本仕様書

```
docs/
└── M4_overview.md                                    # 本ファイル（M4マイルストーン概要仕様書）
```
