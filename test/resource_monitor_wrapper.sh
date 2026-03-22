#!/bin/bash
# リソース監視ラッパースクリプト
#
# センチネルファイル (.stop_rosbag) が作成されたら監視を停止して終了する。
# nav-test が航法完了後にセンチネルを作成することで、resource-monitor サービスを
# 外部コマンドなしに正常停止させる。
#
# ファイル:
#   /root/test/results/.stop_rosbag            : nav-test が停止を要求するセンチネル (rosbag と共用)
#   /root/test/results/.resource_monitor_stopped: resource-monitor が停止完了を通知するフラグ

set -e

RESULTS_DIR="/root/test/results"
CSV_PATH="${RESULTS_DIR}/resources.csv"
SENTINEL="${RESULTS_DIR}/.stop_rosbag"
DONE_FLAG="${RESULTS_DIR}/.resource_monitor_stopped"

# 前回の状態をクリーンアップ
rm -f "${CSV_PATH}" "${DONE_FLAG}"
mkdir -p "${RESULTS_DIR}"

echo "[resource-monitor] リソース監視を開始します: ${CSV_PATH}"
python3 /root/test/resource_monitor.py \
  --output "${CSV_PATH}" \
  --interval 1.0 &
MONITOR_PID=$!

# センチネルファイルが作成されるまで待機
while [ ! -f "${SENTINEL}" ]; do
  if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then
    echo "[resource-monitor] エラー: resource_monitor.py が予期せず終了しました"
    exit 1
  fi
  sleep 0.5
done

echo "[resource-monitor] センチネル検知。監視を停止します..."
kill "${MONITOR_PID}" 2>/dev/null || true
wait "${MONITOR_PID}" 2>/dev/null || true

touch "${DONE_FLAG}"
echo "[resource-monitor] 監視を正常停止しました: ${CSV_PATH}"
