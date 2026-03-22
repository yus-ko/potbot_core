#!/bin/bash
# リソース監視ラッパースクリプト
#
# センチネルファイル (.stop_rosbag) が作成されたら監視を停止して終了する。
# nav-test が航法完了後にセンチネルを作成することで、resource-monitor サービスを
# 外部コマンドなしに正常停止させる。
#
# ファイル:
#   /root/test/results/.stop_rosbag               : nav-test が停止を要求するセンチネル (rosbag と共用)
#   /root/test/results/.resource_monitor_stopped  : resource-monitor が停止完了を通知するフラグ
#   /root/test/results/.current_bag_name          : rosbag-record が書き出したbag名

set -e

RESULTS_DIR="/root/test/results"
SENTINEL="${RESULTS_DIR}/.stop_rosbag"
DONE_FLAG="${RESULTS_DIR}/.resource_monitor_stopped"
BAG_NAME_FILE="${RESULTS_DIR}/.current_bag_name"

# 前回の状態をクリーンアップ（古いCSVは削除しない）
rm -f "${DONE_FLAG}"
mkdir -p "${RESULTS_DIR}"

# rosbag-record が書き出したbag名を待機
echo "[resource-monitor] bag名ファイルを待機中: ${BAG_NAME_FILE}"
WAIT_COUNT=0
MAX_WAIT=60
while [ ! -f "${BAG_NAME_FILE}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "[resource-monitor] 警告: bag名ファイルが見つかりません。タイムスタンプでフォールバック"
    BAG_NAME="resources_$(date +%Y_%m_%d-%H_%M_%S)"
    break
  fi
  sleep 0.5
done

if [ -f "${BAG_NAME_FILE}" ]; then
  BAG_NAME=$(cat "${BAG_NAME_FILE}")
fi

CSV_PATH="${RESULTS_DIR}/${BAG_NAME}.csv"

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

# 最新のCSVへのシンボリックリンクを更新
ln -sfn "${BAG_NAME}.csv" "${RESULTS_DIR}/latest.csv"
echo "[resource-monitor] シンボリックリンクを更新しました: ${RESULTS_DIR}/latest.csv -> ${BAG_NAME}.csv"

touch "${DONE_FLAG}"
echo "[resource-monitor] 監視を正常停止しました: ${CSV_PATH}"
