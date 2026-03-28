# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ブランチルール（potbot_core固有）
- マージ先は `humble` ブランチ（`main` ではない）。

## ドキュメント
`docs/Milestones/` にマイルストーン単位で格納。各マイルストーンは `M{番号}/Overview.md` + `T-{番号}_*.md`（チケット詳細）で構成される。
コードに変更を加える場合はドキュメントを作成し、内容をコードと一致させる。

## パッケージ構成

`potbot_core` は以下5パッケージから構成される：

| パッケージ | 役割 |
|---|---|
| `potbot_lib` | ROS非依存のコアアルゴリズム実装（APF、PID、経路計画、フィルタ等） |
| `potbot_ros` | potbot_lib の ROS 2 Lifecycle ノードラッパー |
| `potbot_plugin` | Nav2 グローバルプランナー・コントローラープラグイン |
| `potbot_behavior_tree` | Nav2 ビヘイビアツリー拡張ノード |
| `potbot_msgs` | カスタムアクション定義（GoalPose.action） |

依存関係の方向：`potbot_lib` → `potbot_ros` → `potbot_plugin` / `potbot_behavior_tree`

※ `potbot_base/` は非アクティブ（`.bak` ファイルのみ）。

## 開発環境

ホスト環境にROS 2はインストールされていない。ビルド・テストはすべてDocker経由で行うこと。

## ビルド・テストコマンド（Docker）

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test

# 全パッケージのビルド + テスト実行（potbot_exampleを除く）
docker compose --profile test run --rm colcon-test

# 個別パッケージのビルド + テスト
docker compose run --rm colcon-test -c "\
  source /opt/ros/humble/setup.bash \
  && cd /root/ros2_ws \
  && colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON \
  && source install/setup.bash \
  && ctest --test-dir build/potbot_lib"

# 特定テストのみ実行
docker compose run --rm colcon-test -c "\
  source /opt/ros/humble/setup.bash \
  && cd /root/ros2_ws \
  && colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON \
  && source install/setup.bash \
  && ctest --test-dir build/potbot_lib -R test_apf_path_planner"
```

テストファイルは `potbot_lib/test/` に配置。`integration/` と `regression/` サブディレクトリもある。
`potbot_plugin/test/` にもプラグインテスト（gtest）がある。

## Docker環境（E2Eテスト用）

`test/` ディレクトリにDocker Compose環境が用意されている：

```bash
cd test/

# Full E2Eパイプライン（Gazebo + Nav2 + テスト実行 + 記録 + 分析）
docker compose --profile pipeline up

# Gazebo + potbot + ドメインブリッジのみ
docker compose up gazebo domain_bridge potbot

# 既存rosbagの分析のみ
docker compose --profile analysis up
```

**サービス構成:** gazebo（Domain 0）、domain_bridge（0↔1転送）、potbot（Domain 1でNav2実行）、rosbag-record、resource-monitor、nav-test、rosbag-analysis

テスト結果は `test/results/{timestamp}/` に保存される（rosbag、resource.csv、trajectory画像等）。

## アーキテクチャ詳細

### potbot_lib クラス階層

```
DiffDriveAgent（基底：x, y, yaw, v, ω 状態モデル）
  ├─ OptimalPathFollower（potbot_lib::controller）
  └─ ApfWaypointController（potbot_lib::controller）← waypoint追従 + APF障害物回避

ArtificialPotentialField
  └─ getForce() で引力・斥力の合成力を解析的に取得可能
```

その他のライブラリクラス：
- `ApfPathPlanner` — APFグリッド経路計画 + Bezier曲線補間
- `Field` — ポテンシャルフィールドのグリッド表現
- `PID` — PID制御器
- `Interpolate` — Bezier曲線ユーティリティ
- `PurePursuit` — Pure Pursuit経路追従
- `KalmanFilter`, `UnscentedKalmanFilter` — フィルタリング
- `ParticleSwarmOptimization` — PSO最適化
- `Utility` — `Point`, `Pose`, `ScanPoint` 構造体とヘルパー関数

### Nav2 プラグイン登録

`potbot_plugin/plugin_nav2_core.xml` に3プラグイン登録：
- `potbot_nav::planner::APF`（nav2_core::GlobalPlanner）— APFグローバル経路計画
- `potbot_nav::controller::OptimalPathFollower`（nav2_core::Controller）— 最適経路追従
- `potbot_nav::controller::HybridApfController`（nav2_core::Controller）— Hybrid A* + APF waypoint制御

`potbot_behavior_tree/behavior_plugin.xml`: RotateToGoalDirection（Behavior）
`potbot_behavior_tree/tree_nodes.xml`: PotbotWait、RotateToGoalDirection（BTノード定義）

### ROS 2 設計パターン

- `potbot_ros` の各クラスは `rclcpp_lifecycle::LifecycleNode` を継承
- Nav2 プラグインは `pluginlib` 経由でロード（`pluginlib_export_plugin_description_file` マクロ使用）
- ビヘイビアツリーノードは Action Server + BT Node + Nav2 Plugin の3層構造
- プラグインクラスは `potbot_lib` のコアクラスをラップし、ROS パラメータ↔内部状態を橋渡しする
