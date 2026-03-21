#!/bin/bash
# Gazebo + Navigation2 E2E テスト実行スクリプト
# 前提: Gazebo + Navigation2 が起動済みであること
# rosbag2 記録 → ナビゲーション → 記録停止 → 解析の一連のパイプラインを実行

set -e

RESULTS_DIR="/root/test/results"
BAG_PATH="${RESULTS_DIR}/rosbag2"
ANALYZE_SCRIPT="/root/test/analyze_rosbag.py"

echo "=== Gazebo + Navigation2 E2E テスト ==="
echo "前提: Gazebo と Navigation2 が起動済みであること"
echo ""

# --- 1. 結果ディレクトリ作成 ---
mkdir -p "${RESULTS_DIR}"

# --- 2. 古い rosbag データを削除 ---
rm -rf "${BAG_PATH}"

# --- 3. rosbag2 記録をバックグラウンドで開始 ---
echo "rosbag2 記録を開始..."
ros2 bag record -o "${BAG_PATH}" /odom /cmd_vel /scan /tf /tf_static &
RECORD_PID=$!

# 記録プロセスを確実に停止するための trap 設定
cleanup() {
  echo ""
  echo "rosbag2 記録を停止 (PID: ${RECORD_PID})..."
  kill "${RECORD_PID}" 2>/dev/null || true
  wait "${RECORD_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# --- 4. 記録開始を確実にするため少し待つ ---
sleep 2

# --- 5. ゴールポーズ送信 ---
echo "ゴールポーズを送信: x=2.0, y=0.5"
NAV_RESULT=0
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" && echo "ナビゲーション成功!" || { echo "ナビゲーション失敗またはタイムアウト"; NAV_RESULT=1; }

# --- 6. 記録プロセスを停止 (trap で自動実行されるが、解析前に明示的に停止) ---
cleanup
trap - EXIT

# --- 7. 解析スクリプト実行 ---
if [ -f "${ANALYZE_SCRIPT}" ]; then
  echo ""
  echo "=== rosbag 解析を実行 ==="
  python3 "${ANALYZE_SCRIPT}" --bag-path "${BAG_PATH}" --output-dir "${RESULTS_DIR}" || echo "警告: 解析スクリプトの実行に失敗しました"
else
  echo "警告: 解析スクリプトが見つかりません: ${ANALYZE_SCRIPT}"
fi

# --- 8. 結果表示 ---
echo ""
echo "=== テスト結果 ==="
echo "rosbag データ: ${BAG_PATH}"
echo "結果: ${RESULTS_DIR}/navigation_result.png"

exit ${NAV_RESULT}
