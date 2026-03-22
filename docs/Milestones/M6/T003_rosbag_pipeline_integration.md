# M6: T-003 テストスクリプトに rosbag 記録・解析パイプラインを統合

## 1. チケット概要

### 目的

E2E テスト実行時に自動的に rosbag2 記録を行い、ナビゲーション完了後にデータ解析と可視化を実行するパイプラインを構築する。

### 背景

T-001 で Docker インフラ、T-002 で解析スクリプトを個別に整備したが、テスト実行時にこれらを手動で組み合わせる必要があった。`run_navigation_test.sh` にパイプラインを統合することで、1コマンドで「記録→ナビゲーション→記録停止→解析→結果出力」の一連の処理を自動実行できるようにする。

### 実装内容

- `test/run_navigation_test.sh` を拡張し、rosbag2 記録・解析パイプラインを組み込み
- `test/README.md` に rosbag 記録と解析の説明セクションを追加

---

## 2. パイプライン処理フロー

`run_navigation_test.sh` は以下の順序で処理を実行する:

| ステップ | 処理内容 | コマンド/詳細 |
|---|---|---|
| 1 | 結果ディレクトリ作成 | `mkdir -p /root/test/results` |
| 2 | 古い rosbag データを削除 | `rm -rf /root/test/results/rosbag2` |
| 3 | rosbag2 記録をバックグラウンドで開始 | `ros2 bag record -o ${BAG_PATH} /odom /cmd_vel /scan /tf /tf_static &` |
| 4 | 記録開始待機 | `sleep 2` |
| 5 | ゴールポーズ送信 | `timeout 120 ros2 action send_goal /navigate_to_pose ...` |
| 6 | 記録プロセスを停止 | `kill ${RECORD_PID}` + `trap EXIT` で確実に停止 |
| 7 | 解析スクリプト実行 | `python3 analyze_rosbag.py --bag-path ... --output-dir ...` |
| 8 | 結果パスを出力 | rosbag データパスと PNG パスを表示 |

### ゴールポーズ

| パラメータ | 値 |
|---|---|
| frame_id | `map` |
| position (x, y, z) | (2.0, 0.5, 0.0) |
| orientation (x, y, z, w) | (0.0, 0.0, 0.0, 1.0) |
| タイムアウト | 120秒 |

### 記録トピック

| トピック名 | メッセージ型 | 用途 |
|---|---|---|
| `/odom` | nav_msgs/msg/Odometry | ロボット位置・速度 |
| `/cmd_vel` | geometry_msgs/msg/Twist | 速度指令 |
| `/scan` | sensor_msgs/msg/LaserScan | LiDAR スキャンデータ |
| `/tf` | tf2_msgs/msg/TFMessage | 座標変換（動的） |
| `/tf_static` | tf2_msgs/msg/TFMessage | 座標変換（静的） |

### エラーハンドリング

- `set -e` によりスクリプト全体で即時エラー終了を有効化
- `trap cleanup EXIT` により、スクリプトが異常終了しても rosbag 記録プロセスを確実に停止
- ナビゲーション失敗時も解析処理は継続実行（途中までの記録データを解析可能）
- 解析スクリプトが見つからない場合は警告メッセージを出力して続行

### スクリプト変数

| 変数名 | 値 | 説明 |
|---|---|---|
| `RESULTS_DIR` | `/root/test/results` | 結果出力先ディレクトリ |
| `BAG_PATH` | `${RESULTS_DIR}/rosbag2` | rosbag2 記録先パス |
| `ANALYZE_SCRIPT` | `/root/test/analyze_rosbag.py` | 解析スクリプトのパス |
| `RECORD_PID` | (動的) | rosbag 記録プロセスの PID |
| `NAV_RESULT` | 0 or 1 | ナビゲーション結果（終了コードに使用） |

---

## 3. 実行コマンド

### Docker Compose による自動実行

```bash
cd test/
docker compose up -d gazebo domain_bridge potbot rosbag-record nav-test
```

`nav-test` コンテナ内で `run_navigation_test.sh` が自動実行され、rosbag 記録→ナビゲーション→解析の一連のパイプラインが動作する。

### 結果の確認

```bash
# 結果画像の確認
ls test/results/navigation_result.png

# rosbag データの確認
ros2 bag info test/results/rosbag2
```

---

## 4. 対応ファイル

- `test/run_navigation_test.sh` — rosbag2 記録・解析パイプラインを統合したテスト実行スクリプト
- `test/README.md` — rosbag 記録と解析の説明セクションを追加
