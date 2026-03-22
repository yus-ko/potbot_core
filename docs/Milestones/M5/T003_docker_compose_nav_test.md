# M5: T-003 Docker Compose nav-test / colcon-test サービス仕様書

## 1. チケット概要

### 目的

`test/docker-compose.yml` に `nav-test` および `colcon-test` サービスを追加し、E2E ナビゲーションテストとユニットテストをコンテナ上で自動実行できる環境を整備する。

### 背景

M0 の `docker-compose.yml` には `gazebo` / `domain_bridge` / `potbot` の3サービスが定義されていたが、テストを自動実行するサービスは存在しなかった。M5 の T-002 で追加した E2E テストスクリプトをコンテナから実行し、さらに全パッケージのビルドとユニットテストを一括実行できるサービスを追加する必要があった。

### 実装内容

- `docker-compose.yml` に `nav-test` サービスを追加（potbot に依存し、30 秒待機後に E2E テストを実行）
- `docker-compose.yml` に `colcon-test` サービスを追加（`test` プロファイル・全パッケージのビルドとテストを一括実行）
- `test/README.md` を新規作成（各テスト種類の説明と実行手順を記述）
- 複数回のバグ修正（コミット `ce3cc73`, `19bc161`, `f024bb6`）

---

## 2. サービス定義

### docker-compose.yml の全サービス構成（M5 追加後）

| サービス | ROS_DOMAIN_ID | 役割 | プロファイル |
|---|---|---|---|
| `gazebo` | 0 | Gazebo シミュレーション実行 | デフォルト |
| `domain_bridge` | 0/1 | トピック転送（clock, tf, tf_static, odom, scan, cmd_vel） | デフォルト |
| `potbot` | 1 | Navigation2 スタック実行 | デフォルト |
| `nav-test` | 1 | E2E テストスクリプト実行 | デフォルト |
| `colcon-test` | — | 全パッケージのビルドとテスト | `test` |

### nav-test サービス定義

```yaml
nav-test:
  <<: *common_setting
  container_name: nav-test
  environment:
    <<: *common_environment
    ROS_DOMAIN_ID: 1
  depends_on:
    - potbot
  volumes:
    - ../../../:/root/ros2_ws
    - ./entrypoint.sh:/entrypoint.sh
    - ./:/root/test
  entrypoint: /entrypoint.sh
  # Nav2 の起動完了（約30秒）を待ってからテストを実行
  command: bash -c "sleep 30 && bash /root/test/run_navigation_test.sh"
```

**設計ポイント:**

- `depends_on: - potbot` で potbot コンテナの起動後に開始する
- `sleep 30` で Nav2 スタックの初期化完了を待機する（環境によって 30〜60 秒に調整可）
- `ROS_DOMAIN_ID: 1` で potbot と同じドメインに接続し、`/navigate_to_pose` アクションにアクセスする
- ワークスペース全体（`../../../`）をマウントしてビルド済みパッケージを参照する

### colcon-test サービス定義

```yaml
colcon-test:
  profiles: [test]
  <<: *common_setting
  container_name: colcon-test
  environment:
    ROS_DISTRO: humble
    TURTLEBOT3_MODEL: waffle_pi
  volumes:
    - ../../../:/root/ros2_ws
  # entrypoint.sh は install/setup.bash を事前に source するため上書き
  entrypoint: /bin/bash
  # potbot_example は Gazebo が必要な E2E テストのみのためスキップ
  # 古いテスト結果を削除してから実行することで test-result に混入しない
  command: >
    -c "source /opt/ros/humble/setup.bash
    && cd /root/ros2_ws
    && colcon build
    && source install/setup.bash
    && rm -rf build/potbot_example/test_results build/potbot_example/Testing
    && colcon test --packages-skip potbot_example
    && colcon test-result --verbose"
```

**設計ポイント:**

- `profiles: [test]` で通常の `docker compose up` には含まれない（`--profile test` 指定時のみ起動）
- `entrypoint: /bin/bash` で `entrypoint.sh` を上書きする。`entrypoint.sh` は `install/setup.bash` を事前に `source` するが、ビルド前は `install/` が存在しないためエラーになる（コミット `ce3cc73` で修正）
- `potbot_example` をスキップする理由: `potbot_example` のテストは Gazebo が必要な E2E テストのみであり、このサービスでは Gazebo が起動していないため
- `rm -rf build/potbot_example/test_results build/potbot_example/Testing` で古いキャッシュを削除する（コミット `19bc161` で修正。古い結果が `colcon test-result` に混入する問題を解消）

---

## 3. 修正履歴

M5 T-003 の実装では以下のバグが発見・修正された：

| コミット | 問題 | 修正内容 |
|---|---|---|
| `ce3cc73` | `entrypoint.sh` が `install/setup.bash` を `source` しようとするがビルド前は存在しないためエラー | `colcon-test` の `entrypoint` を `/bin/bash` に上書き |
| `19bc161` | 古い `potbot_example` のテスト結果が `colcon test-result` に混入する | ビルド後・テスト前に `build/potbot_example/test_results` と `build/potbot_example/Testing` を削除 |
| `f024bb6` | `run_navigation_test.sh` 内でホストの絶対パスがハードコードされていた | スクリプト内のパスをコンテナ内パス（`/root/test/...`）に統一 |
| `8de735a` | `test/` ディレクトリにテスト実行手順の説明がなかった | `test/README.md` を新規作成 |

---

## 4. 起動コマンド

### E2E ナビゲーションテスト（nav-test）

```bash
cd /path/to/potbot_core/test

# 全サービスを起動（Gazebo + potbot + nav-test）
docker compose up -d

# nav-test のログをリアルタイム確認
docker compose logs -f nav-test

# テスト完了後に停止
docker compose down
```

### ユニットテスト一括実行（colcon-test）

```bash
cd /path/to/potbot_core/test

# colcon-test サービスのみ起動（test プロファイルが必要）
docker compose --profile test up colcon-test

# ログ確認
docker compose logs colcon-test
```

colcon-test の実行ステップ:

1. `source /opt/ros/humble/setup.bash` — ROS 2 環境の初期化
2. `colcon build` — 全パッケージのビルド
3. `source install/setup.bash` — ビルド済みパッケージの読み込み
4. `rm -rf build/potbot_example/test_results build/potbot_example/Testing` — 古いキャッシュ削除
5. `colcon test --packages-skip potbot_example` — テスト実行（potbot_ros のユニットテストを含む）
6. `colcon test-result --verbose` — テスト結果の詳細表示

---

## 5. 対応ファイル

```
test/
├── docker-compose.yml                   # nav-test・colcon-testサービス追加（M5で拡張）
└── README.md                            # テスト実行ガイド（M5で新規作成）
```
