# T-003: ApfWaypointController gtest追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-003 |
| タイトル | ApfWaypointControllerのgtestを追加 |
| マイルストーン | M11 |
| ステータス | 完了 |

---

## 概要

`potbot_lib/test/test_apf_waypoint_controller.cpp` に `ApfWaypointController` のユニットテストを追加する。

---

## テストケース一覧

| テスト名 | 内容 |
|---|---|
| `Constructor` | デフォルト構築が成功すること |
| `EmptyPathIsPathEmpty` | パス未設定時に `isPathEmpty()=true` |
| `SetGlobalPath` | パス設定後に `isPathEmpty()=false` |
| `WaypointAdvances` | ロボットをwaypoint近傍に置くとインデックスが進む |
| `ReachedGoal` | ロボットをゴール近傍に置くと `reachedGoal()=true` |
| `VelocityWithinLimits` | `v ∈ [0, v_max]`, `|omega| ∈ [0, omega_max]` |
| `ForwardTargetProducesPositiveV` | ゴールが正面にある場合 `v > 0` |
| `ObstacleRepulsionChangesOutput` | 障害物ありとなしで出力が異なる |
| `ConvergenceSimulation` | 500ステップ以内にゴール到達する収束テスト |

---

## ビルド・実行コマンド

```bash
cd /home/rtx3090/potbot/ros2_ws

colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

ctest --test-dir build/potbot_lib -R test_apf_waypoint_controller --verbose
```

---

## 追加した CMakeLists.txt 設定

```cmake
ament_add_gtest(test_apf_waypoint_controller test/test_apf_waypoint_controller.cpp)
target_include_directories(test_apf_waypoint_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(test_apf_waypoint_controller potbot_lib_utility)
```
