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
#   /root/test/results/.current_run_dir           : rosbag-record が書き出した実行フォルダ名

set -e

RESULTS_DIR="/root/test/results"
SENTINEL="${RESULTS_DIR}/.stop_rosbag"
DONE_FLAG="${RESULTS_DIR}/.resource_monitor_stopped"
RUN_DIR_FILE="${RESULTS_DIR}/.current_run_dir"

# 前回の状態をクリーンアップ（古い実行フォルダは削除しない）
rm -f "${DONE_FLAG}"
mkdir -p "${RESULTS_DIR}"

# このスクリプトの起動時刻を記録（これより新しいファイルのみ今回の実行として受け入れる）
SCRIPT_START=$(date +%s)

# rosbag-record がこの実行向けに書き出した .current_run_dir を待機
# 古い実行が残したファイルを誤読しないよう、スクリプト起動後に作成されたファイルのみ受け入れる
echo "[resource-monitor] 今回の実行フォルダファイルを待機中: ${RUN_DIR_FILE}"
WAIT_COUNT=0
MAX_WAIT=120
while true; do
  if [ -f "${RUN_DIR_FILE}" ]; then
    FILE_MTIME=$(stat -c %Y "${RUN_DIR_FILE}" 2>/dev/null || echo 0)
    if [ "${FILE_MTIME}" -ge "${SCRIPT_START}" ]; then
      break  # このスクリプト起動後に作成されたファイル（今回の実行）
    fi
  fi
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "[resource-monitor] 警告: 実行フォルダファイルが見つかりません。タイムスタンプでフォールバック"
    TIMESTAMP="fallback_$(date +%Y_%m_%d-%H_%M_%S)"
    mkdir -p "${RESULTS_DIR}/${TIMESTAMP}"
    RUN_DIR="${RESULTS_DIR}/${TIMESTAMP}"
    CSV_PATH="${RUN_DIR}/resources.csv"
    break
  fi
  sleep 0.5
done

if [ -z "${TIMESTAMP}" ]; then
  TIMESTAMP=$(cat "${RUN_DIR_FILE}")
  RUN_DIR="${RESULTS_DIR}/${TIMESTAMP}"
  CSV_PATH="${RUN_DIR}/resources.csv"
fi

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
