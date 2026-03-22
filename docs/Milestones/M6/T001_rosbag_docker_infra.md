# M6: T-001 rosbag記録・解析用 Docker インフラ整備

## 1. チケット概要

### 目的

rosbag2 記録と Python 解析の実行基盤を Docker Compose に追加し、E2E テスト実行時のデータ記録・解析を可能にする。

### 背景

M5 で構築した E2E テストパイプライン（Gazebo + Nav2 + nav-test コンテナ）は、ナビゲーションの成否を判定できるが、ロボットの挙動（軌跡・速度プロファイル）を定量的に検証する手段がなかった。rosbag2 によるトピック記録と、rosbags Python ライブラリによるオフライン解析を Docker サービスとして追加することで、テスト結果の可視化・回帰検証の基盤を整える。

### 実装内容

- `test/Dockerfile` に Python 解析パッケージ（rosbags, matplotlib, numpy）のインストールを追加
- `test/docker-compose.yml` に `rosbag-record` サービスと `rosbag-analysis` サービスを追加
- `test/.gitignore` に `results/` ディレクトリを追加し、記録データをバージョン管理対象外にする

---

## 2. Docker サービス構成

### rosbag-record サービス

| 項目 | 値 |
|---|---|
| コンテナ名 | `rosbag-record` |
| ROS_DOMAIN_ID | 1（potbot と同一ドメイン） |
| 依存サービス | `potbot` |
| 記録先 | `/root/test/results/rosbag2` |
| 記録トピック | `/odom`, `/cmd_vel`, `/scan`, `/tf`, `/tf_static` |
| コマンド | `ros2 bag record -o /root/test/results/rosbag2 /odom /cmd_vel /scan /tf /tf_static` |

### rosbag-analysis サービス

| 項目 | 値 |
|---|---|
| コンテナ名 | `rosbag-analysis` |
| プロファイル | `analysis`（明示的に指定しない限り起動しない） |
| 依存サービス | `nav-test` |
| entrypoint | `/bin/bash` |
| コマンド | `python3 /root/test/analyze_rosbag.py --bag-path /root/test/results/rosbag2 --output-dir /root/test/results` |

### Dockerfile 追加パッケージ

```dockerfile
RUN pip3 install rosbags matplotlib numpy
```

- `rosbags` — rosbag2 (sqlite3形式) の読み込み・デシリアライズ
- `matplotlib` — グラフ生成
- `numpy` — 数値処理（matplotlib の依存でもある）

### .gitignore 追加

```
results/
```

---

## 3. 実行コマンド

### rosbag 記録サービスを含めてテスト実行

```bash
cd test/
docker compose up -d gazebo domain_bridge potbot rosbag-record nav-test
```

### 手動で解析のみ実行（記録済みデータに対して）

```bash
cd test/
docker compose --profile analysis run --rm rosbag-analysis
```

---

## 4. 対応ファイル

- `test/Dockerfile` — Python 解析パッケージ（rosbags, matplotlib, numpy）のインストール追加
- `test/docker-compose.yml` — `rosbag-record` / `rosbag-analysis` サービス定義の追加
- `test/.gitignore` — `results/` を追加
