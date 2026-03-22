#!/bin/bash
# Gazebo + Navigation2 E2E テスト実行スクリプト
# 前提: Gazebo + Navigation2 + rosbag-record (rosbag_record_wrapper.sh) が起動済みであること
#
# フロー:
#   1. 実行フォルダファイル (.current_run_dir) が書き出されるまで待機
#   2. ナビゲーション実行（アクションサーバー接続 + bt_navigator active 待機）
#   3. センチネルファイル作成 → rosbag-record を停止
#   4. rosbag-record・resource-monitor の停止完了を待機
#   5. rosbag2 解析 → 実行フォルダ内に navigation_result.png 生成
#   6. 終了（--abort-on-container-exit で他サービスも停止）

set -e

RESULTS_DIR="/root/test/results"
RUN_DIR_FILE="${RESULTS_DIR}/.current_run_dir"
ANALYZE_SCRIPT="/root/test/analyze_rosbag.py"
SENTINEL="/root/test/results/.stop_rosbag"
DONE_FLAG="/root/test/results/.rosbag_stopped"
RESOURCE_DONE_FLAG="/root/test/results/.resource_monitor_stopped"

echo "=== Gazebo + Navigation2 E2E テスト ==="
echo "前提: Gazebo / Navigation2 / rosbag-record サービスが起動済みであること"
echo ""

# --- 1. 結果ディレクトリ作成 ---
mkdir -p "${RESULTS_DIR}"

# --- 2. 実行フォルダファイルが書き出されるまで待機 ---
echo "実行フォルダファイルを待機中: ${RUN_DIR_FILE}"
WAIT_COUNT=0
MAX_WAIT=60
while [ ! -f "${RUN_DIR_FILE}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "警告: 実行フォルダファイルが見つかりません"
    break
  fi
  sleep 0.5
done

TIMESTAMP=""
if [ -f "${RUN_DIR_FILE}" ]; then
  TIMESTAMP=$(cat "${RUN_DIR_FILE}")
else
  echo "エラー: 実行フォルダファイルが見つかりません。テストを中断します"
  exit 1
fi

RUN_DIR="${RESULTS_DIR}/${TIMESTAMP}"
BAG_PATH="${RUN_DIR}/rosbag2"
RESOURCES_CSV="${RUN_DIR}/resources.csv"
PNG_OUTPUT="${RUN_DIR}/navigation_result.png"
echo "実行フォルダ: ${RUN_DIR}"

# --- 3. ゴールポーズ送信（アクションサーバー + bt_navigator active を検知してから実行） ---
# 初期位置・ゴール・タイムアウトは waffle_pi.yaml の navigation_runner.ros__parameters で管理
echo "ナビゲーションを開始します（設定: waffle_pi.yaml の navigation_runner パラメーター）"
NAV_RESULT=0
python3 /root/test/run_navigation.py \
  --ros-args --params-file /root/test/waffle_pi.yaml \
  && echo "ナビゲーション成功!" \
  || { echo "ナビゲーション失敗またはタイムアウト"; NAV_RESULT=1; }

# --- 4. センチネルファイルを作成して rosbag-record に停止を通知 ---
echo "rosbag-record に停止を通知..."
touch "${SENTINEL}"

# --- 5. rosbag-record・resource-monitor の停止完了を待機 ---
echo "rosbag-record の停止完了を待機中..."
WAIT_COUNT=0
MAX_WAIT=30  # 最大30秒待機
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

# --- 6. rosbag2 解析 ---
if [ -f "${ANALYZE_SCRIPT}" ] && [ -d "${BAG_PATH}" ]; then
  echo ""
  echo "=== rosbag 解析を実行 ==="
  ANALYZE_ARGS="--bag-path ${BAG_PATH} --output-image ${PNG_OUTPUT}"
  if [ -f "${RESOURCES_CSV}" ]; then
    ANALYZE_ARGS="${ANALYZE_ARGS} --resources-csv ${RESOURCES_CSV}"
    echo "リソースCSV検出: ${RESOURCES_CSV} -> CPU/メモリパネルを追加"
  fi
  python3 "${ANALYZE_SCRIPT}" ${ANALYZE_ARGS} \
    && echo "解析完了: ${PNG_OUTPUT}" \
    || echo "警告: 解析スクリプトの実行に失敗しました"
else
  echo "警告: rosbag2 データまたは解析スクリプトが見つかりません"
fi

# --- 7. 結果表示 ---
echo ""
echo "=== テスト結果 ==="
echo "実行フォルダ: ${RUN_DIR}"
echo "  rosbag データ: ${BAG_PATH}"
echo "  リソースCSV:   ${RESOURCES_CSV}"
echo "  結果図:        ${PNG_OUTPUT}"
echo "latest シンボリックリンク: ${RESULTS_DIR}/latest -> ${TIMESTAMP}"

exit ${NAV_RESULT}
