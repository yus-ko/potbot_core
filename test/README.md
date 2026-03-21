# potbot_core テスト実行ガイド

このディレクトリには potbot_core のテスト関連ファイルが含まれています。

## ディレクトリ構成

```
test/
├── docker-compose.yml           # テスト用 Docker Compose 設定
├── entrypoint.sh                # コンテナエントリーポイント
├── bridge_config.yaml           # ROS Domain Bridge 設定
├── burger.yaml                  # Navigation2 パラメータ
├── turtlebot3_navigation.launch.py  # Nav2 起動ランチファイル
├── navigation2.rviz             # RViz 設定
├── run_navigation_test.sh       # E2E ナビゲーションテストスクリプト
└── test_navigation_pipeline.py  # launch_testing ベースの E2E テスト
```

---

## テストの種類

| テスト | 内容 | 実行方法 |
|--------|------|---------|
| potbot_ros ユニットテスト | ROS 型変換関数 29 ケース | `colcon test` |
| E2E ナビゲーション（手動） | ゴール発行 → 到着確認 | `run_navigation_test.sh` |
| E2E ナビゲーション（自動） | Docker Compose で一括実行 | `docker compose up -d` |
| colcon ビルド＋テスト | 全パッケージのビルド＋テスト | `docker compose --profile test` |

---

## 1. potbot_ros ユニットテスト

ROS 2 環境が利用可能な場合にホスト上で直接実行できます。

```bash
cd /home/rtx3090/potbot/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select potbot_ros
colcon test --packages-select potbot_ros
colcon test-result --verbose
```

---

## 2. E2E ナビゲーションテスト（Docker Compose）

### 前提条件

- Docker および Docker Compose がインストール済みであること
- `ros/potbot:humble` イメージがビルド済みであること
  ```bash
  cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test
  docker compose build
  ```
- X11 転送が有効であること（Gazebo の GUI 表示を行う場合）
  ```bash
  xhost +local:docker
  ```

### 起動方法

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test

docker compose up -d
```

以下のコンテナが順番に起動します：

| コンテナ | 役割 | 備考 |
|---------|------|------|
| `gazebo` | Gazebo シミュレーション（Domain 0） | turtlebot3_with_garage.launch.py |
| `domain_bridge` | Domain 0 ↔ Domain 1 のトピック中継 | clock, tf, odom, scan, cmd_vel |
| `potbot` | Navigation2 スタック（Domain 1） | use_sim_time:=true |
| `nav-test` | E2E テストスクリプト実行 | Nav2 起動後 30 秒待機してから実行 |

### テスト結果の確認

```bash
# nav-test コンテナのログをリアルタイム確認
docker compose logs -f nav-test

# 全コンテナのログを確認
docker compose logs

# テスト完了後に停止
docker compose down
```

### 期待される出力（成功時）

```
=== Gazebo + Navigation2 E2E テスト ===
前提: Gazebo と Navigation2 が起動済みであること

ゴールポーズを送信: x=2.0, y=0.5
...
ナビゲーション成功!
```

### Nav2 起動待ち時間の調整

環境によって Nav2 の起動に時間がかかる場合は `docker-compose.yml` の `sleep` 秒数を調整してください。

```yaml
# docker-compose.yml
nav-test:
  command: bash -c "sleep 30 && bash /root/test/run_navigation_test.sh"
  #                         ↑ 環境に応じて調整（目安: 30〜60秒）
```

---

## 3. E2E ナビゲーションテスト（手動）

Gazebo と Navigation2 を別々のターミナルで起動してテストを実行する方法です。

### ターミナル 1: Gazebo 起動

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test
docker compose up gazebo domain_bridge
```

### ターミナル 2: Navigation2 起動

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test
docker compose up potbot
```

### ターミナル 3: テスト実行

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test
docker compose run --rm nav-test bash /root/test/run_navigation_test.sh
```

または、ホスト上で直接実行（ROS 2 環境が必要）：

```bash
bash /home/rtx3090/potbot/ros2_ws/src/potbot_core/test/run_navigation_test.sh
```

---

## 4. colcon ビルド＋テスト（全パッケージ）

`test` プロファイルを使用して、すべての potbot パッケージのビルドとテストを実行します。

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test

docker compose --profile test up colcon-test
```

実行内容：
1. `colcon build` — 全パッケージのビルド
2. `colcon test` — 全パッケージのテスト実行
3. `colcon test-result --verbose` — テスト結果の表示

### 結果の確認

```bash
docker compose logs colcon-test
```

---

## トラブルシューティング

### Gazebo が起動しない

```bash
# X11 転送を許可
xhost +local:docker

# DISPLAY 変数を確認
echo $DISPLAY
```

### Nav2 が起動しない

```bash
# potbot コンテナのログを確認
docker compose logs potbot

# entrypoint.sh が install/setup.bash を参照できているか確認
docker compose exec potbot ls /root/ros2_ws/install/setup.bash
```

### ナビゲーションがタイムアウトする

- `run_navigation_test.sh` のタイムアウト（デフォルト 120 秒）を延長する
- `burger.yaml` の `xy_goal_tolerance` / `yaw_goal_tolerance` を確認する
- Gazebo のシミュレーションが正常に動作しているか確認する
