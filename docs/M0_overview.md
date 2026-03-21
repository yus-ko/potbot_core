# M0 マイルストーン概要仕様書 — テスト環境整備

| 項目 | 内容 |
|---|---|
| マイルストーン | M0 |
| タイトル | テスト環境整備 |
| ステータス | 完了 |
| 完了日 | 2026-03-21 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M0 は potbot_core プロジェクトにおけるテスト基盤の整備を目的としたマイルストーンである。

具体的には以下の2点を達成する：

1. **コアライブラリのユニットテスト作成**（T-001）: `potbot_lib` の各クラスに対して網羅的な gtest ベースのユニットテストを新規作成し、アルゴリズムの正確性を継続的に検証できる体制を整える。
2. **単体テスト用 Docker 環境の構築**（T-002）: ROS 2 Humble 環境を再現可能な Docker 構成を `test/` ディレクトリに整備し、CI やローカル環境を問わず単体テストを実行できるようにする。

---

## 2. 背景

### ROS 2 移行との関係

potbot_core は ROS 1 (Melodic) から ROS 2 (Humble) へ移行中のロボットナビゲーションシステムである。移行に伴い以下の変更が行われた：

- `potbot_lib`: ヘッダーファイルを `.h` から `.hpp` に変更、ROS 依存を除去してコアライブラリをクリーンアップ
- `potbot_ros`: ROS 2 対応のラッパークラスを新規作成
- `potbot_plugin`: Nav2 プラグインで APF パスプランナーを統合

これらの変更によりアーキテクチャが大きく変化したため、**リグレッションを防止しアルゴリズムの正確性を保証するユニットテストが不可欠**となった。

### テストが必要な理由

- `potbot_lib` はロボットの運動モデル・経路計画・制御アルゴリズムを担う中核ライブラリであり、バグが直接ロボットの動作に影響する
- ROS 2 移行後のリファクタリングで既存ロジックが意図せず変化するリスクがある
- 開発者が異なる環境（ローカル PC / CI / Docker）で同一の結果を得られる再現性が必要である

---

## 3. スコープ

### 対象パッケージ

| パッケージ | 対象 | 理由 |
|---|---|---|
| `potbot_lib` | **対象** | ROS 非依存のコアアルゴリズム。外部依存なしでユニットテスト可能 |
| `potbot_ros` | 対象外 | ROS 2 ランタイムが必要であり、ユニットテストの難易度が高い |
| `potbot_plugin` | 対象外 | Nav2 フレームワークへの依存がある |
| `potbot_behavior_tree` | 対象外 | 同上 |
| `potbot_msgs` | 対象外 | メッセージ定義のみ |

### テスト対象クラス（T-001）

| クラス | テストファイル | テスト件数 |
|---|---|---|
| `Utility` (Point/Pose/関数群) | `test_utility.cpp` | 27件 |
| `DiffDriveAgent` | `test_diff_drive_agent.cpp` | 17件 |
| `PID` | `test_pid.cpp` | 16件 |
| `Field` | `test_field.cpp` | 20件 |
| `ArtificialPotentialField` | `test_artificial_potential_field.cpp` | 19件 |
| `ApfPathPlanner` | `test_apf_path_planner.cpp` | 11件 |
| **合計** | 6ファイル | **116件** |

### 未テストクラス（スコープ外）

- `OptimalPathFollower`: 経路追従制御器（複雑な依存関係のため今回は対象外）
- `Interpolate`: Bezier曲線ユーティリティ関数群（今後のマイルストーンで対応予定）

---

## 4. チケット一覧

### T-001: potbot_lib ユニットテスト新規作成

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | potbot_lib のユニットテストを新規作成 |
| コミット | `a9b940b` |
| ステータス | 完了 |

**実装内容:**

- `potbot_lib` の6クラスに対応した gtest ベースのテストファイルを新規作成（計1465行）
- `potbot_lib/CMakeLists.txt` に `BUILD_TESTING=ON` 時の gtest 設定を追加
- 全116テストが合格済み

**ビルド・実行コマンド:**

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws  # potbot リポジトリルートから
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# 特定テスト実行
ctest --test-dir build/potbot_lib -R test_apf_path_planner
ctest --test-dir build/potbot_lib -R test_artificial_potential_field
ctest --test-dir build/potbot_lib -R test_diff_drive_agent
ctest --test-dir build/potbot_lib -R test_field
ctest --test-dir build/potbot_lib -R test_pid
ctest --test-dir build/potbot_lib -R test_utility
```

**対応ファイル:**

- `potbot_lib/test/test_utility.cpp`
- `potbot_lib/test/test_diff_drive_agent.cpp`
- `potbot_lib/test/test_pid.cpp`
- `potbot_lib/test/test_field.cpp`
- `potbot_lib/test/test_artificial_potential_field.cpp`
- `potbot_lib/test/test_apf_path_planner.cpp`
- `potbot_lib/CMakeLists.txt`（BUILD_TESTING 設定追加）

---

### T-002: potbot_core 単体テスト用 Docker 環境追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| タイトル | potbot_core 単体テスト用 Docker 環境を追加 |
| コミット | `93da37b` |
| ステータス | 完了 |

**実装内容:**

- `test/` ディレクトリに Docker 環境一式を整備（8ファイル・1221行）
- ROS 2 Humble + Nav2 + Turtlebot3 の再現可能なテスト環境を構築
- ROS_DOMAIN_ID 0（Gazebo）↔ 1（potbot）間のトピック転送設定を含む

**Docker サービス構成:**

| サービス | ROS_DOMAIN_ID | 役割 |
|---|---|---|
| `gazebo` | 0 | Gazebo シミュレーション実行 |
| `domain_bridge` | 0/1 | トピック転送（clock, tf, tf_static, odom, scan, cmd_vel） |
| `potbot` | 1 | ナビゲーション実行 |

**起動コマンド:**

```bash
cd test/

# 全サービス起動
docker-compose up gazebo domain_bridge potbot

# potbot のみ（外部 Gazebo 使用時）
docker-compose up potbot
```

**対応ファイル:**

- `test/Dockerfile` — ROS Humble + Nav2 + Turtlebot3 イメージ定義
- `test/docker-compose.yml` — 3サービス構成（gazebo / domain_bridge / potbot）
- `test/bridge_config.yaml` — ROS_DOMAIN_ID 0↔1 トピック転送設定
- `test/burger.yaml` — Nav2 パラメータ（AMCL, costmap, planner 含む・413行）
- `test/entrypoint.sh` — コンテナ起動スクリプト
- `test/navigation2.rviz` — RViz 設定ファイル（647行）
- `test/turtlebot3_navigation.launch.py` — ナビゲーション起動ファイル（66行）
- `test/.gitignore` — Docker 生成物の除外設定

---

## 5. 完了基準

M0 を完了とみなすための基準は以下の通りである。すべての項目が達成済みであることを確認した。

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | `potbot_lib` の主要6クラスに対してユニットテストが作成されていること | 完了 |
| 2 | `colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON` でビルドが成功すること | 完了 |
| 3 | `ctest --test-dir build/potbot_lib` で全116テストが合格すること | 完了 |
| 4 | `test/` ディレクトリに Docker 環境一式が整備されていること | 完了 |
| 5 | `docker-compose up` で ROS 2 Humble + Nav2 環境が起動できること | 完了 |
| 6 | bridge_config.yaml による ROS ドメイン間トピック転送が設定されていること | 完了 |

---

## 6. 関連ファイル一覧

### ユニットテスト（T-001）

```
potbot_lib/
├── CMakeLists.txt                        # BUILD_TESTING 設定追加
└── test/
    ├── test_utility.cpp                  # Point/Pose/utility関数テスト (27件)
    ├── test_diff_drive_agent.cpp         # 差動駆動ロボット運動学テスト (17件)
    ├── test_pid.cpp                      # PID制御器テスト (16件)
    ├── test_field.cpp                    # ポテンシャルフィールドグリッドテスト (20件)
    ├── test_artificial_potential_field.cpp  # APFアルゴリズムテスト (19件)
    └── test_apf_path_planner.cpp         # APF経路計画テスト (11件)
```

### Docker テスト環境（T-002）

```
test/
├── .gitignore                            # Docker生成物の除外設定
├── Dockerfile                            # ROS Humble + Nav2 + Turtlebot3イメージ
├── docker-compose.yml                    # gazebo/domain_bridge/potbot 3サービス構成
├── bridge_config.yaml                    # ROS_DOMAIN_ID 0↔1 トピック転送設定
├── burger.yaml                           # Nav2パラメータ (413行)
├── entrypoint.sh                         # コンテナ起動スクリプト
├── navigation2.rviz                      # RViz設定ファイル (647行)
└── turtlebot3_navigation.launch.py       # ナビゲーション起動ファイル (66行)
```

### 本仕様書

```
docs/
└── M0_overview.md                        # 本ファイル（M0マイルストーン概要仕様書）
```
