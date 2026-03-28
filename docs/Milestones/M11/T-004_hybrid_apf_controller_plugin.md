# T-004: HybridApfController Nav2 Controllerプラグイン実装

| 項目 | 内容 |
|---|---|
| チケット番号 | T-004 |
| タイトル | HybridApfControllerをNav2プラグインとして追加 |
| マイルストーン | M11 |
| ステータス | 完了 |

---

## 概要

`potbot_lib::controller::ApfWaypointController` を Nav2 の `nav2_core::Controller` インターフェースでラップした `HybridApfController` プラグインを実装する。

グローバルプランナーが生成したパスをwaypointとして受け取り、コストマップから障害物を抽出してリアルタイムに速度指令を計算する。

---

## 変更ファイル

- `potbot_plugin/include/potbot_plugin/hybrid_apf_controller.hpp`（新規）
- `potbot_plugin/src/hybrid_apf_controller.cpp`（新規）
- `potbot_plugin/plugin_nav2_core.xml`（修正）
- `potbot_plugin/CMakeLists.txt`（修正）
- `potbot_plugin/test/test_hybrid_apf_controller.cpp`（新規）

---

## プラグイン情報

| 項目 | 内容 |
|---|---|
| クラス名 | `potbot_nav::controller::HybridApfController` |
| 基底クラス | `nav2_core::Controller` |
| プラグインXML | `plugin_nav2_core.xml` |

---

## パラメータ

| パラメータ名 | デフォルト | 説明 |
|---|---|---|
| `k_att` | 1.0 | APF引力ゲイン |
| `k_rep` | 2.0 | APF斥力ゲイン |
| `d_th` | 0.5 | 斥力有効距離 [m] |
| `k_v` | 0.5 | 速度ゲイン |
| `k_omega` | 2.0 | 角速度ゲイン |
| `v_max` | 0.22 | 最大並進速度 [m/s] |
| `omega_max` | 1.5 | 最大角速度 [rad/s] |
| `waypoint_tolerance` | 0.2 | waypoint到達閾値 [m] |
| `goal_tolerance` | 0.05 | ゴール到達閾値 [m] |
| `obstacle_cost_threshold` | 200.0 | 障害物と判定するコスト値 |
| `max_obstacle_distance` | 2.0 | 障害物として考慮する最大距離 [m] |

---

## 主要メソッド実装概要

### configure()

`nav2_util::declare_parameter_if_not_declared()` で全パラメータを宣言・取得し、`controller_.setParams()` に渡す。

### setPlan()

`potbot_lib::utility::get_path()` で `nav_msgs::msg::Path` を `std::vector<Pose>` に変換して `controller_.setGlobalPath()` に渡す。

### computeVelocityCommands()

1. `costmap_ros_->getRobotPose()` でロボット位置取得
2. `extractObstaclesFromCostmap()` でコストマップから障害物抽出
3. ロボット状態（x, y, yaw）を `controller_` に設定
4. `controller_.computeCommand()` で速度計算
5. `v`, `omega` を `TwistStamped` に詰めて返す

### extractObstaclesFromCostmap()

コストマップの全セルを走査し、コスト値が `obstacle_cost_threshold_` 以上かつロボットからの距離が `max_obstacle_distance_` 以内のセルをワールド座標に変換して障害物リストに追加する。

---

## Nav2 YAML設定例

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "potbot_nav::controller::HybridApfController"
      k_att: 1.0
      k_rep: 2.0
      d_th: 0.5
      k_v: 0.5
      k_omega: 2.0
      v_max: 0.22
      omega_max: 1.5
      waypoint_tolerance: 0.2
      goal_tolerance: 0.05
      obstacle_cost_threshold: 200.0
      max_obstacle_distance: 2.0
```
