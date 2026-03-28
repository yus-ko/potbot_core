#!/bin/bash
# ランダムゴールナビゲーションテスト実行スクリプト
# 前提: Gazebo + Navigation2 + rosbag-record (rosbag_record_wrapper.sh) が起動済みであること
#
# フロー:
#   1. 実行フォルダファイル (.current_run_dir) が書き出されるまで待機
#   2. 結果ディレクトリ確認
#   3. パラメータファイルを実行フォルダにコピー
#   4. ランダムナビゲーション実行
#   5. センチネルファイル作成 → rosbag-record を停止
#   6. rosbag-record・resource-monitor の停止完了を待機
#   7. 結果表示
#   8. 終了

set -e

RESULTS_DIR="/root/test/results"
RUN_DIR_FILE="${RESULTS_DIR}/.current_run_dir"
PARAMS_FILE="${PARAMS_FILE:-/root/test/config/waffle_pi.yaml}"
SENTINEL="/root/test/results/.stop_rosbag"
DONE_FLAG="/root/test/results/.rosbag_stopped"
RESOURCE_DONE_FLAG="/root/test/results/.resource_monitor_stopped"

echo "=== ランダムゴールナビゲーションテスト ==="
echo "前提: Gazebo / Navigation2 / rosbag-record サービスが起動済みであること"
echo ""

# --- 1. 結果ディレクトリ作成 ---
mkdir -p "${RESULTS_DIR}"

SCRIPT_START=$(date +%s)

# --- 2. 今回の実行フォルダファイルが書き出されるまで待機 ---
echo "今回の実行フォルダファイルを待機中: ${RUN_DIR_FILE}"
WAIT_COUNT=0
MAX_WAIT=120
while true; do
  if [ -f "${RUN_DIR_FILE}" ]; then
    FILE_MTIME=$(stat -c %Y "${RUN_DIR_FILE}" 2>/dev/null || echo 0)
    if [ "${FILE_MTIME}" -ge "${SCRIPT_START}" ]; then
      break
    fi
  fi
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "エラー: 実行フォルダファイルが見つかりません。テストを中断します"
    exit 1
  fi
  sleep 0.5
done

TIMESTAMP=$(cat "${RUN_DIR_FILE}")
RUN_DIR="${RESULTS_DIR}/${TIMESTAMP}"
echo "実行フォルダ: ${RUN_DIR}"

# --- 3. パラメータファイルを実行フォルダにコピー ---
if [ -f "${PARAMS_FILE}" ]; then
  cp "${PARAMS_FILE}" "${RUN_DIR}/$(basename "${PARAMS_FILE}")"
  echo "パラメータファイルをコピーしました: ${RUN_DIR}/$(basename "${PARAMS_FILE}")"
else
  echo "警告: パラメータファイルが見つかりません: ${PARAMS_FILE}"
fi

# --- 4. ランダムナビゲーション実行 ---
# results_dir を実行フォルダに設定し、CSV・サマリをそこに保存
echo "ランダムナビゲーションを開始します"
NAV_RESULT=0
python3 /root/test/scripts/random_navigation.py \
  --ros-args --params-file "${PARAMS_FILE}" \
  -p results_dir:="${RUN_DIR}" \
  && echo "ランダムナビゲーション完了!" \
  || { echo "ランダムナビゲーション異常終了"; NAV_RESULT=1; }

# --- 5. センチネルファイルを作成して rosbag-record に停止を通知 ---
echo "rosbag-record に停止を通知..."
touch "${SENTINEL}"

# --- 6. rosbag-record・resource-monitor の停止完了を待機 ---
echo "rosbag-record の停止完了を待機中..."
WAIT_COUNT=0
MAX_WAIT=30
while [ ! -f "${DONE_FLAG}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "警告: rosbag-record の停止確認がタイムアウトしました"
    break
  fi
  sleep 1
done
echo "rosbag-record 停止完了"

echo "resource-monitor の停止完了を待機中..."
WAIT_COUNT=0
while [ ! -f "${RESOURCE_DONE_FLAG}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "警告: resource-monitor の停止確認がタイムアウトしました"
    break
  fi
  sleep 1
done
echo "resource-monitor 停止完了"

# --- 7. 結果表示 ---
echo ""
echo "=== テスト結果 ==="
echo "実行フォルダ: ${RUN_DIR}"
echo "  パラメータ:    ${RUN_DIR}/$(basename "${PARAMS_FILE}")"
echo "  結果CSV:       ${RUN_DIR}/navigation_results.csv"
echo "  サマリJSON:    ${RUN_DIR}/summary_latest.json"
echo "  rosbag データ: ${RUN_DIR}/rosbag2"
echo "  リソースCSV:   ${RUN_DIR}/resources.csv"
echo "latest シンボリックリンク: ${RESULTS_DIR}/latest -> ${TIMESTAMP}"

# --- 8. rosbag-record・resource-monitor に終了許可を通知 ---
touch "${RESULTS_DIR}/.nav_test_done"
echo "全サービスに終了を通知しました"

exit ${NAV_RESULT}
