# M5: T-002 E2E テストパイプライン仕様書

## 1. チケット概要

### 目的

Gazebo シミュレーションと Navigation 2 スタックを組み合わせた E2E（End-to-End）テストパイプラインを構築し、ゴールポーズ発行からゴール到着までの一連のナビゲーションを自動検証できる仕組みを整備する。

### 背景

M0 で整備した Docker 環境は Gazebo + Nav2 の起動には対応していたが、ナビゲーションの成否を自動的に確認するテストスクリプトが存在しなかった。Nav2 プラグイン（`APF`, `OptimalPathFollower`）の変更がロボットの実際の動作に悪影響を与えないことを継続的に確認するため、E2E テストパイプラインが必要となった。

### 実装内容

- `test/run_navigation_test.sh`: `ros2 action send_goal` を用いたゴール発行スクリプト（タイムアウト 120 秒）
- `test/test_navigation_pipeline.py`: `launch_testing` ベースの E2E テストクラス（2テストケース）
- `test/navigation_test_launch.py`: テスト用ランチファイル
- `test/test_navigation_e2e.py`: E2E テスト補助スクリプト
- コミット `39fb2a9` でファイルを potbot_core リポジトリ内の `test/` ディレクトリへ移動・整備

---

## 2. パイプライン構成

### テストの流れ

```
[Docker Compose]
    |
    +-- gazebo コンテナ (Domain 0)
    |     turtlebot3_with_garage.launch.py
    |
    +-- domain_bridge コンテナ (Domain 0/1)
    |     clock, tf, tf_static, odom, scan, cmd_vel を中継
    |
    +-- potbot コンテナ (Domain 1)
    |     turtlebot3_navigation.launch.py (use_sim_time:=true)
    |
    +-- nav-test コンテナ (Domain 1)
          sleep 30 （Nav2 起動待ち）
          run_navigation_test.sh を実行
```

### テスト設定値

| パラメータ | 値 | 説明 |
|---|---|---|
| ゴール位置 | x=2.0, y=0.5 | ナビゲーション目標座標 |
| ゴール許容誤差 | 0.3 m | `GOAL_TOLERANCE`（test_navigation_pipeline.py） |
| ナビゲーションタイムアウト | 120 秒 | ゴール到着までの最大待ち時間 |
| Nav2 起動待ち時間 | 30〜60 秒 | 環境によって調整（docker-compose.yml の `sleep` 値） |

---

## 3. テストファイル詳細

### run_navigation_test.sh

CLI でゴールを発行し、成否を標準出力で確認するシェルスクリプト。

```bash
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" && echo "ナビゲーション成功!" || echo "ナビゲーション失敗またはタイムアウト"
```

### test_navigation_pipeline.py

`launch_testing` ベースのテストクラス。2つのテストケースを含む。

| テスト名 | 検証内容 |
|---|---|
| `test_navigate_to_pose_action_available` | `/navigate_to_pose` アクションサーバーが起動から最大 60 秒以内に利用可能になること |
| `test_full_navigation_cycle` | AMCL 初期位置設定 → ゴール送信 → ゴール到着（STATUS_SUCCEEDED）の一連のサイクルが 120 秒以内に完了すること |

---

## 4. 実行手順

### 前提条件

- Docker および Docker Compose がインストール済みであること
- `ros/potbot:humble` イメージがビルド済みであること
  ```bash
  cd /path/to/potbot_core/test
  docker compose build
  ```
- X11 転送が有効であること（Gazebo の GUI 表示を行う場合）
  ```bash
  xhost +local:docker
  ```

### 自動実行（全コンテナ一括起動）

```bash
cd /path/to/potbot_core/test

# 全サービスを起動（バックグラウンド）
docker compose up -d

# nav-test ログをリアルタイム確認
docker compose logs -f nav-test

# テスト完了後に停止
docker compose down
```

### 手動実行（分割起動）

```bash
# ターミナル 1: Gazebo + ドメインブリッジ起動
docker compose up gazebo domain_bridge

# ターミナル 2: Navigation2 起動
docker compose up potbot

# ターミナル 3: テスト実行
docker compose run --rm nav-test bash /root/test/run_navigation_test.sh
```

### 期待される出力（成功時）

```
=== Gazebo + Navigation2 E2E テスト ===
前提: Gazebo と Navigation2 が起動済みであること

ゴールポーズを送信: x=2.0, y=0.5
...
ナビゲーション成功!
```

---

## 5. トラブルシューティング

### ナビゲーションがタイムアウトする

- `docker-compose.yml` の `nav-test` サービスの `sleep` 秒数を増やす（目安: 30〜60秒）
- `potbot` コンテナのログで Nav2 起動状況を確認する
  ```bash
  docker compose logs potbot
  ```
- `burger.yaml` の `xy_goal_tolerance` / `yaw_goal_tolerance` を確認する

### Gazebo が起動しない

```bash
# X11 転送を許可
xhost +local:docker

# DISPLAY 変数を確認
echo $DISPLAY
```

---

## 6. 対応ファイル

```
test/
├── run_navigation_test.sh               # CLIゴール発行スクリプト（タイムアウト120秒）
├── test_navigation_pipeline.py          # launch_testingベースE2Eテスト（2件）
├── navigation_test_launch.py            # テスト用ランチファイル
└── test_navigation_e2e.py               # E2Eテスト補助スクリプト
```
