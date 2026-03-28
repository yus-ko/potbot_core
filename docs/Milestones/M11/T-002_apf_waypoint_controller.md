# T-002: ApfWaypointController クラス実装 (potbot_lib)

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| タイトル | ApfWaypointControllerクラスをpotbot_libに追加 |
| マイルストーン | M11 |
| ステータス | 完了 |

---

## 概要

グローバルパスのwaypoint追従とAPF斥力回避を組み合わせた制御クラス `ApfWaypointController` を `potbot_lib` に追加する。

`DiffDriveAgent` を継承し、`computeCommand()` を呼び出すことで `v`（並進速度）と `omega`（角速度）が設定される。

---

## 変更ファイル

- `potbot_lib/include/potbot_lib/apf_waypoint_controller.hpp`（新規）
- `potbot_lib/src/apf_waypoint_controller.cpp`（新規）
- `potbot_lib/CMakeLists.txt`（修正）

---

## クラス設計

```
namespace potbot_lib::controller
class ApfWaypointController : public DiffDriveAgent
```

### 継承関係

```
DiffDriveAgent
  └── ApfWaypointController
```

### 主要メソッド

| メソッド | 説明 |
|---|---|
| `setGlobalPath(path)` | グローバルパスを設定し、waypointインデックスを0にリセット |
| `setObstacles(obstacles)` | 障害物リストを設定 |
| `setParams(...)` | 全パラメータを一括設定（APFも更新） |
| `computeCommand()` | APF力に基づいてv, omegaを計算 |
| `reachedGoal()` | ゴールへの到達判定 |
| `isPathEmpty()` | パスが空かどうか |
| `getCurrentWaypointIndex()` | 現在のwaypointインデックス |
| `getCurrentWaypoint()` | 現在のwaypoint座標 |

### パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `k_att_` | 1.0 | 引力ゲイン |
| `k_rep_` | 1.0 | 斥力ゲイン |
| `d_th_` | 0.5 | 斥力有効距離 [m] |
| `k_v_` | 0.5 | 速度ゲイン |
| `k_omega_` | 2.0 | 角速度ゲイン |
| `v_max_` | 0.3 | 最大並進速度 [m/s] |
| `omega_max_` | 1.5 | 最大角速度 [rad/s] |
| `waypoint_tolerance_` | 0.2 | waypoint到達閾値 [m] |
| `goal_tolerance_` | 0.05 | ゴール到達閾値 [m] |

---

## 制御アルゴリズム

### computeCommand()

1. `isPathEmpty()` または `reachedGoal()` の場合は `v=0, omega=0` にして return
2. `updateWaypoint()` でwaypoint進行判定
3. 現在waypointの位置に対してAPF力ベクトルを計算
4. `desired_heading = atan2(fy, fx)` で目標方向を算出
5. `heading_error = normalize(desired_heading - yaw)` で角度誤差を計算
6. `v = min(k_v * |F|, v_max)` で速度を算出（負値は0にクリップ）
7. `omega = clamp(k_omega * heading_error, -omega_max, omega_max)` で角速度を算出

### updateWaypoint()

- 現在waypointまでの距離を計算
- 最終waypoint: `goal_tolerance_`、それ以外: `waypoint_tolerance_` を閾値として使用
- 距離 < 閾値 かつ 最終でない場合は `waypoint_index_++`

### computeForce()

1. `apf_.setRobot(x, y)` でロボット位置設定
2. `apf_.clearObstacles()` で障害物クリア
3. `apf_.setObstacle(obs.x, obs.y)` で各障害物を登録
4. `apf_.getForce(x, y, wx, wy, fx, fy)` で力ベクトル取得
