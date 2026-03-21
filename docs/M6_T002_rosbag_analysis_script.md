# M6: T-002 rosbag2 解析・可視化スクリプト作成

## 1. チケット概要

### 目的

rosbags Python モジュールを使用して rosbag2 データを解析し、ロボットの移動軌跡と速度の時系列データを可視化するスクリプトを作成する。

### 背景

T-001 で Docker インフラに rosbag2 記録機能を追加したが、記録されたデータを解析・可視化する手段がなかった。rosbags ライブラリを使用したオフライン解析スクリプトを作成し、ナビゲーション結果を PNG 画像として出力することで、テスト結果の視覚的な確認と回帰検証を可能にする。

### 実装内容

- `test/analyze_rosbag.py` を新規作成（158行）
- rosbag2 (sqlite3形式) の読み込み、/odom と /cmd_vel トピックのデシリアライズ、3パネル図の生成・保存

---

## 2. スクリプト仕様

### ファイル構成

| ファイル | 行数 | 概要 |
|---|---|---|
| `test/analyze_rosbag.py` | 158行 | rosbag2 解析・可視化メインスクリプト |

### コマンドライン引数

| 引数 | 必須 | デフォルト | 説明 |
|---|---|---|---|
| `--bag-path` | はい | — | rosbag2 ディレクトリのパス |
| `--output-dir` | いいえ | bag-path の親ディレクトリ | 出力 PNG 保存先ディレクトリ |

### 処理フロー

1. コマンドライン引数の解析（`argparse`）
2. rosbag2 ディレクトリの存在確認（存在しない場合はエラー終了）
3. 出力ディレクトリの作成（`mkdir -p` 相当）
4. `rosbags.rosbag2.Reader` で rosbag2 を開き、全メッセージを走査
5. `rosbags.serde.deserialize_cdr` でメッセージをデシリアライズ
6. `/odom`（nav_msgs/msg/Odometry）から位置座標 (x, y) を抽出
7. `/cmd_vel`（geometry_msgs/msg/Twist）から線速度 (linear.x) と角速度 (angular.z) を抽出
8. matplotlib で3パネル図を生成
9. `{output_dir}/navigation_result.png` として保存（dpi=150）

### 出力図の構成

| パネル | 内容 | 軸ラベル |
|---|---|---|
| パネル1 | XY軌跡プロット | X [m] / Y [m] |
| パネル2 | 線速度の時系列グラフ | Time [s] / Linear Velocity [m/s] |
| パネル3 | 角速度の時系列グラフ | Time [s] / Angular Velocity [rad/s] |

パネル1 の詳細:
- 青線: ロボットの移動軌跡
- 緑丸: スタート地点（記録開始時の /odom 座標）
- 赤三角: ゴール地点 (2.0, 0.5)
- アスペクト比: equal（縦横等倍）

### 関数一覧

| 関数名 | 引数 | 戻り値 | 説明 |
|---|---|---|---|
| `parse_args()` | — | `argparse.Namespace` | コマンドライン引数の解析 |
| `read_rosbag(bag_path)` | `str` | `(odom_data, cmd_vel_data)` | rosbag2 からトピックデータを読み込み |
| `create_figure(odom_data, cmd_vel_data)` | タプル2つ | `matplotlib.Figure` | 3パネルの図を生成 |
| `main()` | — | — | メイン処理（引数解析→読み込み→可視化→保存） |

### 依存ライブラリ

| ライブラリ | 用途 |
|---|---|
| `rosbags` | rosbag2 (sqlite3形式) の読み込みとメッセージデシリアライズ |
| `matplotlib` | グラフ生成（Agg バックエンド使用） |
| `argparse` | コマンドライン引数解析（標準ライブラリ） |
| `pathlib` | パス操作（標準ライブラリ） |

---

## 3. 実行コマンド

```bash
python3 analyze_rosbag.py --bag-path /root/test/results/rosbag2 --output-dir /root/test/results
```

出力:
```
/root/test/results/navigation_result.png
```

---

## 4. 対応ファイル

- `test/analyze_rosbag.py` — rosbag2 解析・可視化スクリプト本体（158行）
