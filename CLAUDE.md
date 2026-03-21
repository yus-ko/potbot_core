# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ドキュメント
`docs/` に格納されている。

## パッケージ構成

`potbot_core` は以下5パッケージから構成される：

| パッケージ | 役割 |
|---|---|
| `potbot_lib` | ROS非依存のコアアルゴリズム実装（APF、PID、経路計画） |
| `potbot_ros` | potbot_lib の ROS 2 Lifecycle ノードラッパー |
| `potbot_plugin` | Nav2 グローバルプランナー・コントローラープラグイン |
| `potbot_behavior_tree` | Nav2 ビヘイビアツリー拡張ノード |
| `potbot_msgs` | カスタムアクション定義（GoalPose.action） |

依存関係の方向：`potbot_lib` → `potbot_ros` → `potbot_plugin` / `potbot_behavior_tree`

## ビルドコマンド

```bash
# ワークスペースルートに移動（必須）
cd /home/rtx3090/potbot/ros2_ws

# 全体ビルド
colcon build

# 個別ビルド（依存順に注意）
colcon build --packages-select potbot_msgs
colcon build --packages-select potbot_lib
colcon build --packages-select potbot_ros
colcon build --packages-select potbot_plugin
colcon build --packages-select potbot_behavior_tree

# 環境設定（ビルド後・ターミナル起動時に必要）
source install/setup.bash
```

## テストコマンド

テストは `potbot_lib` のみに定義されている（6つのgtest実行形式）。

```bash
# テスト付きでビルド
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

テストファイルは `potbot_lib/test/` に配置。各テストはgtest形式で実装されている。

## Docker環境（単体テスト用）

`test/` ディレクトリにDocker環境が用意されている：

```bash
cd test/

# Gazebo + potbot + ドメインブリッジを起動
docker-compose up gazebo domain_bridge potbot

# または potbot のみ（外部Gazebo使用時）
docker-compose up potbot
```

- **gazebo**: ROS_DOMAIN_ID=0 でシミュレーション実行
- **domain_bridge**: Domain 0↔1 のトピック転送（clock, tf, tf_static, odom, scan, cmd_vel）
- **potbot**: ROS_DOMAIN_ID=1 でナビゲーション実行

## アーキテクチャ詳細

### potbot_lib クラス構成

- `DiffDriveAgent` — 差動駆動ロボットの運動モデル（状態: x, y, yaw, v, ω）
- `ArtificialPotentialField` — 引力・斥力フィールドを用いたAPFアルゴリズム
- `ApfPathPlanner` — APFによるグリッドベース経路計画 + Bezier曲線補間
- `Field` — ポテンシャルフィールドのグリッド表現
- `OptimalPathFollower` — 経路追従制御器
- `PID` — PID制御器
- `Interpolate` — Bezier曲線ユーティリティ
- `Utility` — Point, Pose, Direction 構造体とヘルパー関数

### Nav2 プラグイン登録

- `plugin_nav2_core.xml`: APF（GlobalPlanner）、OptimalPathFollower（Controller）
- `behavior_plugin.xml`: RotateToGoalDirection（Behavior）
- `tree_nodes.xml`: PotbotWait、RotateToGoalDirection（BTノード定義）

### ROS 2 設計パターン

- `potbot_ros` の各クラスは `rclcpp_lifecycle::LifecycleNode` を継承
- Nav2 プラグインは `pluginlib` 経由でロード（`pluginlib_export_plugin_description_file` マクロ使用）
- ビヘイビアツリーノードは Action Server + BT Node + Nav2 Plugin の3層構造

## ブランチルール（potbot_core固有）

マージ先は `humble` ブランチ（`main` ではない）。
