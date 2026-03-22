# potbot_core テスト実行ガイド

このディレクトリには potbot_core のテスト関連ファイルが含まれています。

## ディレクトリ構成

```
test/
├── docker-compose.yml           # テスト用 Docker Compose 設定
├── Dockerfile                   # コンテナイメージ定義
├── config/
│   ├── bridge_config.yaml       # ROS Domain Bridge 設定（Domain 0↔1）
│   ├── waffle_pi.yaml           # Navigation2 パラメータ + ナビゲーション実行パラメータ
│   └── navigation2.rviz         # RViz 設定
├── launch/
│   ├── turtlebot3_navigation.launch.py  # Nav2 起動ランチファイル（テスト用）
│   └── navigation_test_launch.py        # launch_testing 対応の E2E テスト起動ファイル
├── scripts/
│   ├── entrypoint.sh            # コンテナエントリーポイント
│   ├── run_navigation_test.sh   # E2E テスト統制スクリプト（Docker 内実行）
│   ├── rosbag_record_wrapper.sh # rosbag2 記録スクリプト（センチネル検知で停止）
│   ├── resource_monitor_wrapper.sh  # リソース監視スクリプト（センチネル検知で停止）
│   ├── run_navigation.py        # ゴール発行・ナビゲーション実行スクリプト
│   ├── resource_monitor.py      # CPU/メモリ使用量を CSV に記録するスクリプト
│   └── analyze_rosbag.py        # rosbag2 解析・可視化スクリプト
├── tests/
│   ├── test_analyze_rosbag.py   # analyze_rosbag.py の単体テスト（pytest）
│   ├── test_navigation_e2e.py   # launch_testing ベースの E2E テスト
│   └── test_navigation_pipeline.py  # Gazebo + Nav2 の統合パイプラインテスト
└── results/                     # テスト結果出力ディレクトリ（gitignore）
    ├── YYYY_MM_DD-HH_MM_SS/     # 実行ごとのタイムスタンプフォルダ
    │   ├── rosbag2/             # rosbag2 記録データ
    │   ├── resources.csv        # CPU/メモリ時系列データ
    │   ├── navigation_result.png  # 軌跡・速度・リソース可視化グラフ
    │   └── waffle_pi.yaml       # 使用したパラメータファイルのコピー（ファイル名は実行時のものに従う）
    └── latest -> YYYY_MM_DD-HH_MM_SS/  # 最新実行へのシンボリックリンク
```

---

## テストの種類

| テスト | 内容 | プロファイル |
|--------|------|-------------|
| E2E ナビゲーション | Gazebo + Nav2 でゴール到達を確認、rosbag 記録・解析まで自動実行 | `pipeline` |
| rosbag 解析のみ | 既存の rosbag データに対して可視化グラフを再生成 | `analysis` |
| 単体テスト（pytest） | `analyze_rosbag.py` の動作検証 | `test` |
| colcon ビルド＋テスト | 全パッケージのビルドとテスト | `test` |

---

## 1. E2E ナビゲーションテスト（Docker Compose）

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

docker compose --profile pipeline up --abort-on-container-exit
```

以下のコンテナが順番に起動します：

| コンテナ | 役割 |
|---------|------|
| `gazebo` | Gazebo シミュレーション（Domain 0、turtlebot3_with_garage） |
| `domain_bridge` | Domain 0 ↔ Domain 1 のトピック中継（clock, tf, odom, scan, cmd_vel） |
| `potbot` | Navigation2 スタック（Domain 1、use_sim_time:=true） |
| `rosbag-record` | テスト中のトピックを rosbag2 で記録、タイムスタンプフォルダに保存 |
| `resource-monitor` | CPU/メモリ使用量を計測して resources.csv に記録 |
| `nav-test` | ゴール発行・到達確認・rosbag 解析の一連のフローを統制 |

### テスト実行フロー

1. `rosbag-record` がタイムスタンプフォルダを作成し、rosbag2 の記録を開始
2. `nav-test` がフォルダ名を確認後、`run_navigation.py` で Nav2 の起動完了を待機してゴール発行
3. ナビゲーション完了後、`nav-test` がセンチネルファイル (`.stop_rosbag`) を作成
4. `rosbag-record` / `resource-monitor` がセンチネルを検知して停止
5. `nav-test` が `analyze_rosbag.py` を実行して `navigation_result.png` を生成

### ナビゲーションパラメータの変更

初期位置・ゴール・タイムアウトは `config/waffle_pi.yaml` の `navigation_runner.ros__parameters` で管理します：

```yaml
navigation_runner:
  ros__parameters:
    initial_pose_x: -2.0   # AMCL 初期位置 X
    initial_pose_y: -0.5   # AMCL 初期位置 Y
    goal_x: 2.0            # ゴール X
    goal_y: 0.5            # ゴール Y
    timeout: 300.0         # タイムアウト秒数
```

### テスト結果の確認

```bash
# nav-test コンテナのログをリアルタイム確認
docker compose logs -f nav-test

# 全コンテナのログを確認
docker compose logs --profile pipeline

# 最新の結果フォルダを確認
ls -la results/latest/
```

テスト完了後、`results/latest/` に以下が生成されます：

| ファイル | 内容 |
|---------|------|
| `rosbag2/` | 記録されたトピックデータ（sqlite3 形式） |
| `resources.csv` | CPU/メモリ使用量の時系列データ |
| `navigation_result.png` | 軌跡・速度・リソース使用量の可視化グラフ |
| `waffle_pi.yaml`（等） | 使用したパラメータファイルのコピー |

### 期待される出力（成功時）

```
=== Gazebo + Navigation2 E2E テスト ===
前提: Gazebo / Navigation2 / rosbag-record サービスが起動済みであること

今回の実行フォルダファイルを待機中: /root/test/results/.current_run_dir
実行フォルダ: /root/test/results/2026_03_22-10_00_00
ナビゲーションを開始します（設定: waffle_pi.yaml の navigation_runner パラメーター）
ゴール到達成功！
ナビゲーション成功!
=== rosbag 解析を実行 ===
解析完了: /root/test/results/2026_03_22-10_00_00/navigation_result.png
```

---

## 2. rosbag 解析のみ実行

既存の rosbag データに対して可視化グラフを再生成します。

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test

docker compose --profile analysis run --rm rosbag-analysis
```

`results/latest/` の rosbag2 データを読み込み、`results/latest/navigation_result.png` を上書き出力します。

---

## 3. 単体テスト（pytest）

`analyze_rosbag.py` の動作を合成データで検証します。

```bash
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test

docker compose --profile test run --rm pytest
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
2. `colcon test` — 全パッケージのテスト実行（`potbot_example` は Gazebo が必要な E2E テストのみのためスキップ）
3. `colcon test-result --verbose` — テスト結果の表示

```bash
# 結果の確認
docker compose logs colcon-test
```

---

## 記録されるトピック

`rosbag_record_wrapper.sh` が以下のトピックを記録します：

| トピック | 内容 |
|---------|------|
| `/odom` | オドメトリ（ロボットの位置・速度） |
| `/cmd_vel` | 速度指令 |
| `/scan` | LiDAR スキャンデータ |
| `/tf` | 座標変換 |
| `/tf_static` | 静的座標変換 |
| `/plan` | グローバル経路計画 |
| `/test/goal_pose` | テスト用ゴール位置（rosbag に記録するためパブリッシュ） |
| `/map` | 占有格子マップ |

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

- `config/waffle_pi.yaml` の `navigation_runner.ros__parameters.timeout` を延長する
- `config/waffle_pi.yaml` の `xy_goal_tolerance` / `yaw_goal_tolerance` を確認する
- Gazebo のシミュレーションが正常に動作しているか確認する

### nav-test が「実行フォルダファイルが見つかりません」で終了する

- `rosbag-record` コンテナが正常に起動しているか確認する
- `rosbag-record` のログで `.current_run_dir` の書き出しが行われているか確認する
  ```bash
  docker compose logs rosbag-record
  ```
