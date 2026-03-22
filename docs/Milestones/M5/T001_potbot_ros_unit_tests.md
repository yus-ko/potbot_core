# M5: T-001 potbot_ros ユニットテスト仕様書

## 1. チケット概要

### 目的

`potbot_ros` パッケージに初めてユニットテストを追加し、ROS 型変換ユーティリティ関数（`potbot_ros/utility.hpp`）の正確性を Google Test で自動検証できる体制を整備する。

### 背景

`potbot_ros/utility.hpp` は `potbot_lib` の内部型（`Point`, `Pose`）と ROS 2 メッセージ型（`geometry_msgs::msg::Point`, `geometry_msgs::msg::Pose` 等）の相互変換を提供するユーティリティヘッダーである。Nav2 プラグイン（`APF`, `OptimalPathFollower`）がこの変換関数に直接依存しているため、誤った変換はナビゲーション動作全体に影響する。

M0 では `potbot_ros` がテスト対象外だったが（ROS 2 ランタイムへの依存が理由）、M5 では `ament_cmake_gtest` を用いることで ROS 2 のビルドシステム上でテストを実行できることを確認し、29 テストケースを新規追加した。

### 実装内容

- `potbot_ros/test/test_ros_utility.cpp` を新規作成（29テストケース）
- `potbot_ros/CMakeLists.txt` に `BUILD_TESTING=ON` 時の `ament_cmake_gtest` 設定を追加

---

## 2. テストファイル構成

| ファイル | 行数 | テストケース数 | テストスイート名 |
|---|---|---|---|
| `test/test_ros_utility.cpp` | 352行 | 29件 | `ROSUtilityTest` |

---

## 3. テストケース一覧

### get_point テスト（5件）

| テスト名 | 検証内容 |
|---|---|
| `GetPointFromXYZ` | `get_point(x, y, z)` が正しい座標を返すこと |
| `GetPointDefaultZero` | 引数なし `get_point()` が原点を返すこと |
| `GetPointFromPotbotPoint` | `potbot_lib::Point` から `geometry_msgs::msg::Point` への変換 |
| `GetPotbotPointFromMsgPoint` | `geometry_msgs::msg::Point` から `potbot_lib::Point` への変換 |
| `GetPointRoundTrip` | `potbot_lib::Point` → ROS メッセージ → `potbot_lib::Point` の往復整合性 |

### get_quat / get_rpy テスト（3件）

| テスト名 | 検証内容 |
|---|---|
| `GetQuatFromRPY` | ゼロ角の場合 w=1, x=y=z=0 のクォータニオンが得られること |
| `GetRPYRoundTrip` | `get_quat(roll, pitch, yaw)` → `get_rpy(q, ...)` の往復整合性 |
| `GetRPYYawOnly` | Yaw 角のみ設定した場合の往復整合性（π/4 rad） |

### get_pose テスト（4件）

| テスト名 | 検証内容 |
|---|---|
| `GetPoseFromXYZRPY` | `get_pose(x, y, z, roll, pitch, yaw)` が正しい位置・向きを返すこと |
| `GetPoseFromPotbotPose` | `potbot_lib::Pose` から `geometry_msgs::msg::Pose` への変換 |
| `GetPotbotPoseFromMsgPose` | `geometry_msgs::msg::Pose` から `potbot_lib::Pose` への変換 |
| `GetPoseRoundTrip` | `potbot_lib::Pose` → ROS メッセージ → `potbot_lib::Pose` の往復整合性 |

### get_distance テスト（3件）

| テスト名 | 検証内容 |
|---|---|
| `GetDistanceKnownValue` | 3-4-5 三角形（既知値: 5.0m）の距離計算 |
| `GetDistanceSamePoint` | 同一点間の距離が 0.0 であること |
| `GetDistance3D` | 3次元距離計算（√3 m の確認） |

### get_map_index / get_map_coordinate テスト（6件）

| テスト名 | 検証内容 |
|---|---|
| `GetMapIndexOrigin` | 原点座標がインデックス 0 に対応すること |
| `GetMapIndexKnownCell` | インデックス 5 のセル座標からインデックス 5 が復元できること |
| `GetMapIndexRowTwo` | インデックス 200 の往復整合性 |
| `GetMapCoordinateOrigin` | インデックス 0 のセル座標が原点 (0.0, 0.0) であること |
| `GetMapCoordinateCell1` | インデックス 1 のセル座標が (0.05, 0.0) であること（resolution=0.05） |
| `GetMapCoordinateRow2` | インデックス 100 のセル座標が (0.0, 0.05) であること |

### get_path / to_msg テスト（2件）

| テスト名 | 検証内容 |
|---|---|
| `GetPathRoundTrip` | `vector<Pose>` → `nav_msgs::msg::Path` → `vector<Pose>` の往復整合性 |
| `ToMsgPosesRoundTrip` | `vector<Pose>` → `vector<PoseStamped>` への変換 |

### color::get_msg テスト（6件）

| テスト名 | 検証内容 |
|---|---|
| `ColorGetMsgRed` | RED カラー (r=1, g=0, b=0, a=1) |
| `ColorGetMsgGreen` | GREEN カラー (r=0, g=1, b=0, a=1) |
| `ColorGetMsgBlue` | BLUE カラー (r=0, g=0, b=1, a=1) |
| `ColorGetMsgYellow` | YELLOW カラー (r=1, g=1, b=0, a=1) |
| `ColorGetMsgWhite` | WHITE カラー (r=1, g=1, b=1, a=1) |
| `ColorGetMsgModulo` | color_id=8 が RED と同一になること（8色でモジュロ折り返し） |

---

## 4. テスト実行コマンド

### ホスト上での実行（ROS 2 環境が必要）

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash

# ビルド
colcon build --packages-select potbot_ros

# テスト実行
colcon test --packages-select potbot_ros

# 結果確認
colcon test-result --verbose
```

### Docker コンテナ上での実行（colcon-test サービス）

```bash
cd /path/to/potbot_core/test
docker compose --profile test up colcon-test
```

実行内容:
1. `colcon build` — 全パッケージのビルド
2. `colcon test --packages-skip potbot_example` — テスト実行（potbot_ros を含む）
3. `colcon test-result --verbose` — テスト結果の表示

---

## 5. 対応ファイル

- `potbot_ros/test/test_ros_utility.cpp` — テスト本体（29件）
- `potbot_ros/CMakeLists.txt` — `ament_cmake_gtest` 設定追加
