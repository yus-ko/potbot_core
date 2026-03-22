#!/bin/bash
# Gazebo + Navigation2 E2E テスト実行スクリプト
# 前提: Gazebo + Navigation2 + rosbag-record サービスが起動済みであること
#
# ナビゲーション実行のみを担当する。rosbag2 の解析は rosbag-record 停止後に
# rosbag-analysis サービスで別途実行すること。

set -e

RESULTS_DIR="/root/test/results"

echo "=== Gazebo + Navigation2 E2E テスト ==="
echo "前提: Gazebo / Navigation2 / rosbag-record サービスが起動済みであること"
echo ""

# --- 1. 結果ディレクトリ作成 ---
mkdir -p "${RESULTS_DIR}"

# --- 2. ゴールポーズ送信（アクションサーバー接続確立まで待機） ---
echo "アクションサーバーを待機してゴールポーズを送信: x=2.0, y=0.5"
NAV_RESULT=0
python3 /root/test/run_navigation.py 2.0 0.5 --timeout 300 \
  && echo "ナビゲーション成功!" \
  || { echo "ナビゲーション失敗またはタイムアウト"; NAV_RESULT=1; }

# --- 3. rosbag-record のフラッシュ時間を確保してから終了 ---
# nav-test 終了 → --abort-on-container-exit で rosbag-record に SIGTERM が届く
# stop_grace_period=15s の間に rosbag-record が metadata.yaml を書き込む
echo "rosbag-record のフラッシュ待機中 (10s)..."
sleep 10

# --- 4. 結果表示 ---
echo ""
echo "=== テスト完了 ==="
echo "rosbag データ: ${RESULTS_DIR}/rosbag2 (rosbag-record 停止後に確定)"
echo "解析は 'docker compose --profile analysis run --rm rosbag-analysis' で実行してください"

exit ${NAV_RESULT}
