# M0: T-002 potbot_core単体テスト用Docker環境仕様書

## 1. チケット概要

### 目的

potbot_coreパッケージの単体テストおよびシステム統合テストをローカル環境やCI環境で再現可能な形で実行するためのDocker環境を整備する。

### 背景

potbot_coreはROS 2 Humble上で動作するロボットナビゲーションシステムである。テストには以下の要素が必要になるため、それぞれをDockerコンテナとして分離管理する：

- Gazeboシミュレーター（ロボット・環境の仮想空間）
- ROS Domain Bridge（シミュレーター空間とナビゲーション空間の通信）
- Navigation 2スタック（potbot_coreプラグインを含む）

### 実装内容

`test/`ディレクトリに以下8ファイルを追加した：

| ファイル | 行数 | 内容 |
|---|---|---|
| `Dockerfile` | 19行 | ROS Humble + Nav2 + Turtlebot3ベースイメージ |
| `docker-compose.yml` | 50行 | 3サービス（gazebo/domain_bridge/potbot）定義 |
| `bridge_config.yaml` | 19行 | ROS Domain 0↔1トピック転送設定 |
| `burger.yaml` | 413行 | Nav2パラメータ（AMCL/costmap/controller等） |
| `entrypoint.sh` | 6行 | ROS環境セットアップスクリプト |
| `navigation2.rviz` | 647行 | RViz2可視化設定 |
| `turtlebot3_navigation.launch.py` | 66行 | ナビゲーション起動スクリプト |
| `.gitignore` | 1行 | Gazeboキャッシュ（`.gazebo/`）除外 |

---

## 2. Docker環境構成図

```
ホストマシン
├── /tmp/.X11-unix  ─────────────────── Xディスプレイ共有（GUI表示用）
└── ros2_ws/  ───────────────────────── ワークスペース（全コンテナが共有）
    └── src/potbot_core/test/

┌──────────────────────────────────────────────────────────────┐
│ Docker Compose ネットワーク（bridge）                          │
│                                                              │
│  ┌──────────────────┐    ┌──────────────────┐               │
│  │  gazebo           │    │  domain_bridge    │               │
│  │  ROS_DOMAIN_ID=0  │    │  （ドメイン変換）  │               │
│  │  Gazeboシミュ起動  │    │                  │               │
│  └──────────────────┘    └──────────────────┘               │
│          │                        │                          │
│          │ Domain 0               │ Domain 0→1 転送           │
│          │  clock, tf, tf_static  │  clock, tf, tf_static    │
│          │  odom, scan            │  odom, scan              │
│          │  ←cmd_vel（逆方向）    │  cmd_vel（逆方向）→       │
│          │                        │                          │
│          └────────────────────────┘                          │
│                                   │                          │
│                                   │ Domain 1                 │
│                          ┌────────────────┐                  │
│                          │  potbot         │                  │
│                          │  ROS_DOMAIN_ID=1│                  │
│                          │  Nav2 + RViz2   │                  │
│                          └────────────────┘                  │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. 各サービスの詳細仕様

### 3.1 共通設定

全サービスは共通のAnchorsで定義された設定を継承する：

| 項目 | 値 |
|---|---|
| ビルドコンテキスト | `test/`（Dockerfile同梱） |
| イメージ名 | `ros/potbot:humble` |
| ネットワークモード | `bridge` |
| stdin_open / tty | `true`（インタラクティブ操作対応） |

共通環境変数：

| 変数名 | 説明 |
|---|---|
| `TERM` | ターミナルタイプ（ホストから継承） |
| `DISPLAY` | X11ディスプレイ番号（ホストから継承） |
| `TURTLEBOT3_MODEL` | `waffle_pi`（使用するTurtlebot3モデル） |

### 3.2 gazeboサービス

| 項目 | 値 |
|---|---|
| コンテナ名 | `gazebo` |
| ROS_DOMAIN_ID | （未設定、デフォルト0） |
| エントリポイント | `/entrypoint.sh` |
| 起動コマンド | `ros2 launch potbot_example turtlebot3_with_garage.launch.py` |

ボリュームマウント：

| ホストパス | コンテナパス | 用途 |
|---|---|---|
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 GUI表示 |
| `test/.gazebo` | `/root/.gazebo` | Gazeboキャッシュ永続化 |
| `ros2_ws/` | `/root/ros2_ws` | ワークスペース共有 |
| `test/entrypoint.sh` | `/entrypoint.sh` | 起動スクリプト |

### 3.3 domain_bridgeサービス

| 項目 | 値 |
|---|---|
| コンテナ名 | `domain_bridge` |
| エントリポイント | （Dockerfileデフォルト） |
| 起動コマンド | `ros2 run domain_bridge domain_bridge /bridge_config.yaml` |

ボリュームマウント：

| ホストパス | コンテナパス | 用途 |
|---|---|---|
| `test/bridge_config.yaml` | `/bridge_config.yaml` | ブリッジ設定 |

### 3.4 potbotサービス

| 項目 | 値 |
|---|---|
| コンテナ名 | `potbot` |
| ROS_DOMAIN_ID | `1` |
| エントリポイント | `/entrypoint.sh` |
| 起動コマンド | `ros2 launch /root/test/turtlebot3_navigation.launch.py use_sim_time:=true` |

ボリュームマウント：

| ホストパス | コンテナパス | 用途 |
|---|---|---|
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 GUI表示 |
| `ros2_ws/` | `/root/ros2_ws` | ワークスペース共有 |
| `test/entrypoint.sh` | `/entrypoint.sh` | 起動スクリプト |
| `test/` | `/root/test` | テスト設定ファイル |

---

## 4. 設定ファイル詳細

### 4.1 Dockerfile

ベースイメージ `ros:humble` に以下パッケージを追加インストール：

| パッケージ | 用途 |
|---|---|
| `ros-humble-rviz2` | 可視化ツール |
| `ros-humble-rqt-*` | ROSデバッグツール群 |
| `ros-humble-navigation2` | Nav2ナビゲーションスタック |
| `ros-humble-turtlebot3-gazebo` | Turtlebot3 Gazeboシミュレーション |
| `ros-humble-turtlebot3-navigation2` | Turtlebot3 Nav2設定 |
| `ros-humble-domain-bridge` | ROS Domainブリッジ |
| `ros-humble-pcl-*` | PCLライブラリ（potbot_rosが依存） |

`.bashrc`に自動追記される設定：
- `source /opt/ros/humble/setup.bash`
- `source /root/ros2_ws/install/setup.bash`
- `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

### 4.2 entrypoint.sh

コンテナ起動時にROS環境を有効化するスクリプト。`set -e`で失敗時即時終了し、環境設定後に渡されたコマンドを`exec "$@"`で実行する：

```bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/ros2_ws/install/setup.bash" --
exec "$@"
```

### 4.3 bridge_config.yaml

ROS Domain 0（Gazebo）からDomain 1（potbot）へのトピック転送設定：

| トピック | メッセージ型 | 転送方向 |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Domain 0 → Domain 1 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Domain 0 → Domain 1 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Domain 0 → Domain 1 |
| `/odom` | `nav_msgs/msg/Odometry` | Domain 0 → Domain 1 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Domain 0 → Domain 1 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Domain 1 → Domain 0（逆方向） |

`cmd_vel`のみ`reversed: True`が設定されており、potbotが出力した速度指令をGazebo側のロボットに送る。

### 4.4 burger.yaml（Nav2パラメータ）

Nav2スタック全体のパラメータを定義する413行の設定ファイル。主要設定を以下に示す：

#### AMCL（自己位置推定）

| パラメータ | 値 | 説明 |
|---|---|---|
| `set_initial_pose` | `True` | 起動時に初期位置を自動設定 |
| `initial_pose.x` | `-2.0` | 初期位置X（Willow Garage座標） |
| `initial_pose.y` | `-0.5` | 初期位置Y |
| `max_particles` | `2000` | パーティクルフィルター最大粒子数 |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | 差動駆動モデル |

#### コントローラーサーバー

| パラメータ | 値 | 説明 |
|---|---|---|
| `controller_frequency` | `20.0 Hz` | 制御ループ周波数 |
| `FollowPath.plugin` | `potbot_nav::controller::OptimalPathFollower` | potbot独自コントローラー |
| `xy_goal_tolerance` | `0.25 m` | 目標到達判定（位置） |
| `yaw_goal_tolerance` | `0.25 rad` | 目標到達判定（角度） |

#### プランナーサーバー

| パラメータ | 値 | 説明 |
|---|---|---|
| `expected_planner_frequency` | `20.0 Hz` | 経路計画周波数 |
| `GridBased.plugin` | `potbot_nav::planner::APF` | potbot独自APFプランナー |

#### ローカルコストマップ

| パラメータ | 値 | 説明 |
|---|---|---|
| `width` / `height` | `3 m` | コストマップサイズ |
| `resolution` | `0.05 m` | グリッド解像度 |
| `robot_radius` | `0.1 m` | ロボット半径 |
| `plugins` | `obstacle_layer, voxel_layer, inflation_layer` | 使用レイヤー |

#### グローバルコストマップ

| パラメータ | 値 | 説明 |
|---|---|---|
| `resolution` | `0.05 m` | グリッド解像度 |
| `robot_radius` | `0.1 m` | ロボット半径 |
| `inflation_radius` | `0.55 m` | 障害物膨張半径 |

### 4.5 turtlebot3_navigation.launch.py

potbotコンテナのNav2スタック起動スクリプト。以下を起動する：

1. `nav2_bringup/bringup_launch.py` — Nav2スタック全体（マップサーバー、AMCL、コントローラー等）
2. `rviz2` — `navigation2.rviz`設定による可視化

起動時引数：

| 引数 | デフォルト値 | 説明 |
|---|---|---|
| `map` | `potbot_example/maps/willowgarage.yaml` | 使用するマップファイル |
| `params_file` | `test/burger.yaml` | Nav2パラメータファイル |
| `use_sim_time` | `false` | シミュレーション時間使用有無 |

---

## 5. ディレクトリ構成

```
ros2_ws/src/potbot_core/
└── test/
    ├── .gitignore                    # .gazebo/キャッシュを除外
    ├── Dockerfile                    # ROS Humble + Nav2 + PCLイメージ定義
    ├── docker-compose.yml            # 3サービス定義
    ├── bridge_config.yaml            # Domain Bridge設定
    ├── burger.yaml                   # Nav2パラメータ（413行）
    ├── entrypoint.sh                 # ROS環境セットアップ
    ├── navigation2.rviz              # RViz2設定
    └── turtlebot3_navigation.launch.py  # ナビゲーション起動スクリプト
```

---

## 6. 起動手順

### 6.1 前提条件

| 要件 | バージョン |
|---|---|
| Docker | 20.10以上推奨 |
| Docker Compose | v2.0以上推奨 |
| ホストOSのX11サーバー | GUIウィンドウ表示に必要 |
| potbot_coreビルド済みワークスペース | `/home/rtx3090/potbot/ros2_ws` |

X11アクセス許可（GUI表示を行う場合）：

```bash
xhost +local:docker
```

### 6.2 全サービス起動（推奨）

```bash
cd ros2_ws/src/potbot_core/test/

# 全サービスを起動（Gazebo + Domain Bridge + Nav2）
docker-compose up gazebo domain_bridge potbot
```

### 6.3 potbotのみ起動（外部Gazebo使用時）

外部でGazeboとDomain Bridgeが既に動作している場合：

```bash
cd ros2_ws/src/potbot_core/test/

docker-compose up potbot
```

### 6.4 初回ビルド

Dockerイメージが存在しない場合は自動ビルドされるが、明示的にビルドする場合：

```bash
docker-compose build
```

### 6.5 コンテナ内でのデバッグ

```bash
# potbotコンテナに接続
docker-compose exec potbot bash

# コンテナ内でROS2トピック確認
ros2 topic list
ros2 topic echo /scan
```

### 6.6 停止・クリーンアップ

```bash
# サービス停止
docker-compose down

# ボリューム・イメージも削除する場合
docker-compose down --rmi all --volumes
```

---

## 7. 注意事項

### ワークスペースのマウント

`ros2_ws/`ディレクトリ全体がコンテナにマウントされるため、ホスト側でビルドした成果物（`install/`）がそのままコンテナ内で利用される。コードを変更した場合はホスト側でリビルドが必要：

```bash
cd ros2_ws/
colcon build --packages-select potbot_lib potbot_ros potbot_plugin
```

### Gazeboキャッシュ

`test/.gazebo/`にGazeboのモデルキャッシュが保存される。このディレクトリは`.gitignore`で除外されている。

### ROS Domain ID

- Domain 0: Gazeboシミュレーション空間
- Domain 1: Nav2/potbotナビゲーション空間

同一ホスト上で複数のROS 2システムを分離するためにDomain IDを使い分ける。`domain_bridge`サービスが両者の通信を中継する。

---

## 8. 関連チケット

| チケット | 内容 |
|---|---|
| M0: T-001 | potbot_libのユニットテスト新規作成 |
| M0: T-002 | 本チケット（Docker環境構築） |
