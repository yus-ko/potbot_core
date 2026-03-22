#!/bin/bash
# Gazebo + Navigation2 E2E テスト実行スクリプト
# 前提: Gazebo + Navigation2 + rosbag-record (rosbag_record_wrapper.sh) が起動済みであること
#
# フロー:
#   1. bag名ファイル (.current_bag_name) が書き出されるまで待機
#   2. ナビゲーション実行（アクションサーバー接続 + bt_navigator active 待機）
#   3. センチネルファイル作成 → rosbag-record を停止
#   4. rosbag-record・resource-monitor の停止完了を待機
#   5. rosbag2 解析 → <bag名>.png 生成 + latest.png シンボリックリンク作成
#   6. 終了（--abort-on-container-exit で他サービスも停止）

set -e

RESULTS_DIR="/root/test/results"
BAG_NAME_FILE="${RESULTS_DIR}/.current_bag_name"
ANALYZE_SCRIPT="/root/test/analyze_rosbag.py"
SENTINEL="/root/test/results/.stop_rosbag"
DONE_FLAG="/root/test/results/.rosbag_stopped"
RESOURCE_DONE_FLAG="/root/test/results/.resource_monitor_stopped"

echo "=== Gazebo + Navigation2 E2E テスト ==="
echo "前提: Gazebo / Navigation2 / rosbag-record サービスが起動済みであること"
echo ""

# --- 1. 結果ディレクトリ作成 ---
mkdir -p "${RESULTS_DIR}"

# --- 2. bag名ファイルが書き出されるまで待機 ---
echo "bag名ファイルを待機中: ${BAG_NAME_FILE}"
WAIT_COUNT=0
MAX_WAIT=60
while [ ! -f "${BAG_NAME_FILE}" ]; do
  WAIT_COUNT=$((WAIT_COUNT + 1))
  if [ "${WAIT_COUNT}" -gt "${MAX_WAIT}" ]; then
    echo "警告: bag名ファイルが見つかりません。デフォルト名を使用します"
    break
  fi
  sleep 0.5
done

BAG_NAME=""
if [ -f "${BAG_NAME_FILE}" ]; then
  BAG_NAME=$(cat "${BAG_NAME_FILE}")
fi

BAG_PATH="${RESULTS_DIR}/${BAG_NAME}"
RESOURCES_CSV="${RESULTS_DIR}/${BAG_NAME}.csv"
PNG_OUTPUT="${RESULTS_DIR}/${BAG_NAME}.png"
echo "bag名: ${BAG_NAME}"

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
  ANALYZE_ARGS="--bag-path ${BAG_PATH} --output-dir ${RESULTS_DIR} --output-image ${PNG_OUTPUT}"
  if [ -f "${RESOURCES_CSV}" ]; then
    ANALYZE_ARGS="${ANALYZE_ARGS} --resources-csv ${RESOURCES_CSV}"
    echo "リソースCSV検出: ${RESOURCES_CSV} -> CPU/メモリパネルを追加"
  fi
  python3 "${ANALYZE_SCRIPT}" ${ANALYZE_ARGS} \
    && echo "解析完了: ${PNG_OUTPUT}" \
    || echo "警告: 解析スクリプトの実行に失敗しました"

  # 最新のPNGへのシンボリックリンクを更新
  if [ -f "${PNG_OUTPUT}" ]; then
    ln -sfn "${BAG_NAME}.png" "${RESULTS_DIR}/latest.png"
    echo "シンボリックリンクを更新しました: ${RESULTS_DIR}/latest.png -> ${BAG_NAME}.png"
  fi
else
  echo "警告: rosbag2 データまたは解析スクリプトが見つかりません"
fi

# --- 7. 結果表示 ---
echo ""
echo "=== テスト結果 ==="
echo "rosbag データ: ${BAG_PATH}"
echo "結果図: ${PNG_OUTPUT}"

exit ${NAV_RESULT}
