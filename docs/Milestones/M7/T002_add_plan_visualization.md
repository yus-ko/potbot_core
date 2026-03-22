# M7: T-002 /plan トピック記録・可視化追加

## チケット概要

| 項目 | 内容 |
|------|------|
| マイルストーン | M7 |
| チケット番号 | T-002 |
| 概要 | rosbag 解析パイプラインに `/plan` トピック（Nav2 グローバル経路）の記録・可視化機能を追加する |

---

## 変更対象ファイル

| ファイル | 変更種別 |
|----------|----------|
| `test/run_navigation_test.sh` | 修正 |
| `test/docker-compose.yml` | 修正 |
| `test/analyze_rosbag.py` | 修正 |
| `docs/M7_T002_add_plan_visualization.md` | 新規作成 |

---

## 変更内容

### 1. `test/run_navigation_test.sh`

`ros2 bag record` コマンドのトピックリストに `/plan` を追加した。

```bash
# 変更前
ros2 bag record -o "${BAG_PATH}" /odom /cmd_vel /scan /tf /tf_static &

# 変更後
ros2 bag record -o "${BAG_PATH}" /odom /cmd_vel /scan /tf /tf_static /plan &
```

### 2. `test/docker-compose.yml`

`rosbag-record` サービスの `command` に `/plan` を追加した。

```yaml
# 変更前
command: ros2 bag record -o /root/test/results/rosbag2 /odom /cmd_vel /scan /tf /tf_static

# 変更後
command: ros2 bag record -o /root/test/results/rosbag2 /odom /cmd_vel /scan /tf /tf_static /plan
```

### 3. `test/analyze_rosbag.py`

#### `parse_args()` 関数

ゴール座標をコマンドライン引数で受け取れるように拡張した。

- `--goal-x`: ゴール地点のX座標（デフォルト: `-1.25`）
- `--goal-y`: ゴール地点のY座標（デフォルト: `3.5`）

#### `read_rosbag()` 関数

`/plan` トピックを読み込む処理を追加した。返り値に `plan_data` を追加。

```python
# 返り値
odom_data, cmd_vel_data, plan_data = read_rosbag(bag_path)

# plan_data の構造
plan_timestamps  # 各 /plan メッセージのタイムスタンプリスト [s]
plan_paths       # 各 /plan メッセージの [(x, y), ...] リスト
```

#### `create_figure()` 関数

- 引数に `plan_data`、`goal_x`、`goal_y` を追加
- パネル構成を `(3, 1)` から `(4, 1)` に変更
- ゴールマーカー座標をパラメータ化（ハードコードから引数経由に変更）

#### `main()` 関数

- `read_rosbag()` の返り値を `odom_data, cmd_vel_data, plan_data` に変更
- `create_figure()` に `plan_data`、`goal_x`、`goal_y` を渡すように変更

---

## /plan トピックのメッセージ構造

`/plan` トピックは `nav_msgs/msg/Path` 型のメッセージを配信する。

```
nav_msgs/msg/Path
  ├── header
  │     ├── stamp
  │     └── frame_id
  └── poses[]          # PoseStamped の配列
        ├── header
        └── pose
              ├── position
              │     ├── x   ← 経路点のX座標
              │     ├── y   ← 経路点のY座標
              │     └── z
              └── orientation
```

アクセス方法:

```python
msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
x = msg.poses[i].pose.position.x
y = msg.poses[i].pose.position.y
```

---

## 4パネル可視化の説明

| パネル | タイトル | 内容 |
|--------|----------|------|
| Panel 1 | Robot Trajectory | オドメトリ軌跡（青線）と計画経路を重ねて表示。最新の計画経路は赤線、古い計画経路は灰色細線で描画。ゴールマーカーは赤三角で表示。 |
| Panel 2 | Linear Velocity (cmd_vel) | 線速度の時系列グラフ |
| Panel 3 | Angular Velocity (cmd_vel) | 角速度の時系列グラフ |
| Panel 4 | Planned Path Points Over Time | 全計画経路点の XY 散布図。点の色はメッセージ受信時刻（viridis カラーマップ）を表す。 |

### Panel 1 の凡例

| 凡例ラベル | 描画スタイル | 意味 |
|-----------|-------------|------|
| Plan (latest) | 赤線 | 最新の /plan メッセージの経路 |
| Plan (old) | 灰色細線 | 古い /plan メッセージの経路群 |
| Trajectory (odom) | 青線 | 実際のロボット軌跡（オドメトリ） |
| Start | 緑丸 | 出発地点 |
| Goal | 赤三角 | ゴール地点 |
