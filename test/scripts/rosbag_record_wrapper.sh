#!/bin/bash
# rosbag2 記録ラッパースクリプト
#
# センチネルファイル (.stop_rosbag) が作成されたら記録を停止して終了する。
# nav-test が航法完了後にセンチネルを作成することで、rosbag-record サービスを
# 外部コマンドなしに正常停止させる。
#
# ファイル:
#   /root/test/results/.stop_rosbag       : nav-test が記録停止を要求するセンチネル
#   /root/test/results/.rosbag_stopped    : rosbag-record が停止完了を通知するフラグ
#   /root/test/results/.current_run_dir   : 今回の実行フォルダ名（他サービスが参照）

set -e

RESULTS_DIR="/root/test/results"
SENTINEL="${RESULTS_DIR}/.stop_rosbag"
DONE_FLAG="${RESULTS_DIR}/.rosbag_stopped"
RUN_DIR_FILE="${RESULTS_DIR}/.current_run_dir"

# タイムスタンプベースの実行フォルダを生成
TIMESTAMP=$(date +%Y_%m_%d-%H_%M_%S)
RUN_DIR="${RESULTS_DIR}/${TIMESTAMP}"
BAG_PATH="${RUN_DIR}/rosbag2"

# 前回の状態をクリーンアップ（古い実行フォルダは削除しない）
# .current_run_dir を先に削除することで他サービスが古い値を読み込まないようにする
NAV_DONE_FLAG="${RESULTS_DIR}/.nav_test_done"
rm -f "${SENTINEL}" "${DONE_FLAG}" "${RUN_DIR_FILE}" "${NAV_DONE_FLAG}"
mkdir -p "${RUN_DIR}"

# 実行フォルダ名を共有ファイルに書き出す（resource-monitor・nav-test が参照）
echo "${TIMESTAMP}" > "${RUN_DIR_FILE}"

echo "[rosbag-record] rosbag2 記録を開始します: ${BAG_PATH}"
BAG_RECORD_ARGS="-o ${BAG_PATH} /odom /cmd_vel /scan /tf /tf_static /plan /test/goal_pose /map /amcl_pose"
if [ -n "${MAX_BAG_SIZE}" ]; then
  BAG_RECORD_ARGS="--max-bag-size ${MAX_BAG_SIZE} ${BAG_RECORD_ARGS}"
  echo "[rosbag-record] 最大バッグサイズ: ${MAX_BAG_SIZE} bytes"
fi
ros2 bag record ${BAG_RECORD_ARGS} &
RECORD_PID=$!

# センチネルファイルが作成されるまで待機
while [ ! -f "${SENTINEL}" ]; do
  # rosbag record プロセスが予期せず終了した場合は異常終了
  if ! kill -0 "${RECORD_PID}" 2>/dev/null; then
    echo "[rosbag-record] エラー: ros2 bag record が予期せず終了しました"
    exit 1
  fi
  sleep 0.5
done

echo "[rosbag-record] センチネル検知。記録を停止します..."
kill "${RECORD_PID}" 2>/dev/null || true
wait "${RECORD_PID}" 2>/dev/null || true

# 最新の実行フォルダへのシンボリックリンクを更新
ln -sfn "${TIMESTAMP}" "${RESULTS_DIR}/latest"
echo "[rosbag-record] シンボリックリンクを更新しました: ${RESULTS_DIR}/latest -> ${TIMESTAMP}"

# 停止完了フラグを作成して nav-test に通知
touch "${DONE_FLAG}"
echo "[rosbag-record] 記録を正常停止しました: ${BAG_PATH}"

# nav-test の解析完了を待機してから終了
# （早期終了すると --abort-on-container-exit が発火して nav-test の解析が中断される）
echo "[rosbag-record] nav-test の完了を待機中..."
WAIT_COUNT=0
MAX_WAIT=300
while [ ! -f "${NAV_DONE_FLAG}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "[rosbag-record] 警告: nav-test 完了待機タイムアウト"
    break
  fi
  sleep 1
done
