# M5 マイルストーン概要仕様書 — E2Eテストパイプライン + potbot_ros ユニットテスト

| 項目 | 内容 |
|---|---|
| マイルストーン | M5 |
| タイトル | E2Eテストパイプライン + potbot_ros ユニットテスト |
| ステータス | 完了 |
| 完了日 | 2026-03-21 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M5 は potbot_core プロジェクトにおけるテスト基盤を拡充するマイルストーンである。

M0 で整備した `potbot_lib` ユニットテストと Docker 環境を基盤として、以下の3点を達成する：

1. **potbot_ros ユニットテスト追加**（T-001）: `potbot_ros` パッケージに初めてユニットテストを追加し、ROS 型変換ユーティリティ関数の正確性を自動検証できる体制を整える。
2. **Gazebo + Nav2 E2E テストパイプライン構築**（T-002）: Gazebo シミュレーションと Navigation 2 を組み合わせた E2E テストパイプラインを構築し、ゴールポーズ発行からゴール到着までの一連のナビゲーションを自動検証できるようにする。
3. **Docker Compose へのテストサービス追加**（T-003）: `test/docker-compose.yml` に `nav-test` および `colcon-test` サービスを追加し、コンテナ上での自動テスト実行環境を整備する。

---

## 2. 背景

### M0 からの継続

M0 では `potbot_lib` のユニットテスト（116件）と基本的な Docker 環境（3サービス：gazebo / domain_bridge / potbot）を整備した。しかし以下の課題が残されていた：

- `potbot_ros` パッケージにユニットテストが存在しなかった。ROS 型変換関数（`get_point`, `get_pose`, `get_quat` 等）のバグが検出されない状態であった。
- ナビゲーション E2E テストが手動実行のみで、自動化されていなかった。
- Docker Compose からテストを一括実行する手段がなかった。

### テスト拡充の必要性

- `potbot_ros/utility.hpp` は ROS メッセージ型と `potbot_lib` 内部型の相互変換を行う重要なユーティリティであり、Nav2 プラグイン（APF・OptimalPathFollower）が直接依存している。
- 実際のロボット動作（Gazebo + Nav2）を通じた E2E 検証を自動化することで、プラグイン変更時のリグレッションを早期検出できる。
- CI やローカル環境で再現可能な形でテストを実行するため、Docker Compose による一括実行環境が必要であった。

---

## 3. スコープ

### 対象パッケージ

| パッケージ | 対象 | 理由 |
|---|---|---|
| `potbot_ros` | **対象**（T-001） | ROS 型変換ユーティリティのユニットテストを追加 |
| `potbot_lib` | 間接対象 | T-001 のテストが `potbot_lib` 型を参照 |
| `potbot_plugin` | **対象**（T-002） | E2E テストで Nav2 プラグインの動作を検証 |
| `potbot_example` | **対象**（T-002） | Gazebo 起動に使用 |
| `potbot_behavior_tree` | 対象外 | 今回スコープ外 |
| `potbot_msgs` | 対象外 | メッセージ定義のみ |

### テスト対象（T-001）

| クラス | テストファイル | テストケース数 |
|---|---|---|
| `potbot_lib::utility` (ROS版) | `test_ros_utility.cpp` | 29件 |

### E2E テスト構成（T-002）

| ファイル | 役割 |
|---|---|
| `test/run_navigation_test.sh` | CLIによるゴール発行スクリプト |
| `test/test_navigation_pipeline.py` | launch_testing ベースの E2E テスト |
| `test/navigation_test_launch.py` | テスト用ランチファイル |
| `test/test_navigation_e2e.py` | E2E テスト補助スクリプト |

---

## 4. チケット一覧

### T-001: potbot_ros ユニットテスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | potbot_ros にユニットテストを追加 |
| コミット | `81dd316` |
| ステータス | 完了 |

**実装内容:**

- `potbot_ros/test/test_ros_utility.cpp` を新規作成（352行・29テストケース）
- `potbot_ros/CMakeLists.txt` に `ament_cmake_gtest` によるテスト設定を追加
- テスト対象: `potbot_ros/utility.hpp` の ROS 型変換関数群（`get_point`, `get_pose`, `get_quat`, `get_rpy`, `get_distance`, `get_map_index`, `get_map_coordinate`, `get_path`, `to_msg`, `color::get_msg`）

---

### T-002: Gazebo + Nav2 E2E テストパイプライン構築

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| タイトル | Gazebo + Nav2 の E2E テストパイプラインを追加 |
| コミット | `8324f6d`, `39fb2a9` |
| ステータス | 完了 |

**実装内容:**

- `run_navigation_test.sh`: `ros2 action send_goal` を用いたゴール発行スクリプトを新規作成（タイムアウト120秒）
- `test_navigation_pipeline.py`: `launch_testing` ベースの E2E テストクラスを新規作成（2テストケース）
- E2E テスト関連ファイルを `potbot_core/test/` ディレクトリへ移動・整備

---

### T-003: Docker Compose nav-test / colcon-test サービス追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-003 |
| タイトル | docker-compose.yml に nav-test と colcon-test サービスを追加 |
| コミット | `643c9fa`, `ce3cc73`, `19bc161`, `f024bb6`, `8de735a` |
| ステータス | 完了 |

**実装内容:**

- `docker-compose.yml` に `nav-test` サービスを追加（potbot に依存し、30秒待機後に `run_navigation_test.sh` を実行）
- `docker-compose.yml` に `colcon-test` サービスを追加（`test` プロファイル・全パッケージのビルドとテストを一括実行）
- `test/README.md` を新規作成（テスト種類の説明と実行手順を記述）
- 複数回のバグ修正: エントリポイントの上書き・古いテスト結果キャッシュ削除・ホストパスのハードコード除去

---

## 5. 完了基準

M5 を完了とみなすための基準は以下の通りである。すべての項目が達成済みであることを確認した。

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | `potbot_ros` に gtest ベースのユニットテストが追加されていること | 完了 |
| 2 | `colcon test --packages-select potbot_ros` で29件のテストが合格すること | 完了 |
| 3 | `test/run_navigation_test.sh` が存在し、ゴール発行スクリプトが実装されていること | 完了 |
| 4 | `test/test_navigation_pipeline.py` が存在し、E2E テストクラスが実装されていること | 完了 |
| 5 | `docker-compose.yml` に `nav-test` サービスが追加されていること | 完了 |
| 6 | `docker-compose.yml` に `colcon-test` サービスが追加されていること | 完了 |
| 7 | `docker compose --profile test up colcon-test` でビルドとテストが一括実行できること | 完了 |
| 8 | `test/README.md` にテスト実行手順が記述されていること | 完了 |

---

## 6. 関連ファイル一覧

### potbot_ros ユニットテスト（T-001）

```
potbot_ros/
├── CMakeLists.txt                        # ament_cmake_gtest 設定追加
└── test/
    └── test_ros_utility.cpp              # ROS型変換ユーティリティテスト (29件)
```

### E2E テストパイプライン（T-002）

```
test/
├── run_navigation_test.sh               # CLIゴール発行スクリプト
├── test_navigation_pipeline.py          # launch_testingベースE2Eテスト
├── navigation_test_launch.py            # テスト用ランチファイル
└── test_navigation_e2e.py               # E2Eテスト補助スクリプト
```

### Docker Compose テストサービス（T-003）

```
test/
├── docker-compose.yml                   # nav-test・colcon-testサービス追加
└── README.md                            # テスト実行ガイド
```

### 本仕様書

```
docs/
├── M5_overview.md                       # 本ファイル（M5マイルストーン概要仕様書）
├── M5_T001_potbot_ros_unit_tests.md     # T-001 詳細仕様書
├── M5_T002_e2e_test_pipeline.md         # T-002 詳細仕様書
└── M5_T003_docker_compose_nav_test.md   # T-003 詳細仕様書
```
