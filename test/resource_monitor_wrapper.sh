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

# rosbag-record が .current_run_dir を書き出すまで待機
# （rosbag_record_wrapper.sh が起動時に削除してから書き直すため、ファイルが消えてから現れるまで待つ）
echo "[resource-monitor] 実行フォルダファイルを待機中: ${RUN_DIR_FILE}"
WAIT_COUNT=0
MAX_WAIT=60
while [ ! -f "${RUN_DIR_FILE}" ]; do
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

# タイムアウトしなかった場合はファイルから読み込む
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
