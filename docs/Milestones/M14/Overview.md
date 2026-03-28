# M14: ランダムゴールナビゲーションテスト

## 目的

ランダムな位置にゴールを設定してナビゲーションを繰り返し実行し、スタックや極端な停留を自動検出するテスト基盤を構築する。

## 背景

既存の `pipeline` プロファイルは固定ゴール1回のみで、特定シナリオの検証に限られていた。実運用を想定すると、様々な位置へのナビゲーションで安定性を統計的に評価する仕組みが必要。

## 設計概要

### ゴール生成
- マップPGM + YAMLを事前解析し、ロボット初期位置を含む連結成分（free cell）からランダムサンプリング
- オプションで矩形範囲フィルタ `goal_region` を指定可能

### ナビゲーション実行
- デフォルト100ゴール逐次実行（`num_goals: 100`、0=無限ループ）
- 現在地から次のゴールへ（位置リセットなし）
- タイムアウト: `max(min_timeout, distance / expected_speed * safety_margin)`
  - デフォルト: `expected_speed: 0.2`, `safety_margin: 3.0`, `min_timeout: 30.0`

### スタック検出
- `/odom` を10秒間隔で監視し、移動距離 < 0.1m で「停留」1回
- 連続3回停留（30秒間動けない）→ ゴールキャンセル、`stuck` として記録、次のゴールへ
- タイムアウトはフォールバック

### 結果記録
- **CSV**: ゴールごとに1行（goal_id, 座標, distance, result, duration, stuck_count, path_length, timeout）
- **サマリ**: 10ゴールごと + 終了時に stdout + `summary_latest.json` 出力
- **rosbag**: `--max-bag-size 1073741824`（1GB分割）

### Docker構成
- 新profile `random-test` を追加
- `nav-random-test` サービスが専用スクリプトを実行
- gazebo, domain_bridge, potbot, rosbag-record, resource-monitor は既存サービスを共有

## 実行方法

```bash
cd test/

# デフォルト（100ゴール、rosbag 1GB分割）
MAX_BAG_SIZE=1073741824 docker compose --profile random-test up --abort-on-container-exit

# 無限ループ（Ctrl+Cで停止）
MAX_BAG_SIZE=1073741824 docker compose --profile random-test up
```

## チケット一覧

| チケット | 内容 | 状態 |
|---------|------|------|
| T-001 | ドキュメント作成 | 完了 |
| T-002 | random_navigation.py 作成 | - |
| T-003 | run_random_navigation_test.sh 作成 | - |
| T-004 | docker-compose.yml 変更 | - |
| T-005 | waffle_pi.yaml / rosbag_record_wrapper.sh 変更 | - |
| T-006 | 動作確認 | - |
| T-007 | 結果収集と改善計画立案 | - |
