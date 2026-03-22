#!/bin/bash
# rosbag2 記録ラッパースクリプト
#
# センチネルファイル (.stop_rosbag) が作成されたら記録を停止して終了する。
# nav-test が航法完了後にセンチネルを作成することで、rosbag-record サービスを
# 外部コマンドなしに正常停止させる。
#
# ファイル:
#   /root/test/.stop_rosbag   : nav-test が記録停止を要求するセンチネル
#   /root/test/.rosbag_stopped: rosbag-record が停止完了を通知するフラグ

set -e

BAG_PATH="/root/test/results/rosbag2"
SENTINEL="/root/test/results/.stop_rosbag"
DONE_FLAG="/root/test/results/.rosbag_stopped"

# 前回の状態をクリーンアップ
rm -f "$SENTINEL" "$DONE_FLAG"
rm -rf "$BAG_PATH"
mkdir -p /root/test/results

echo "[rosbag-record] rosbag2 記録を開始します: ${BAG_PATH}"
ros2 bag record -o "$BAG_PATH" /odom /cmd_vel /scan /tf /tf_static /plan /test/goal_pose &
RECORD_PID=$!

# センチネルファイルが作成されるまで待機
while [ ! -f "$SENTINEL" ]; do
  # rosbag record プロセスが予期せず終了した場合は異常終了
  if ! kill -0 "$RECORD_PID" 2>/dev/null; then
    echo "[rosbag-record] エラー: ros2 bag record が予期せず終了しました"
    exit 1
  fi
  sleep 0.5
done

echo "[rosbag-record] センチネル検知。記録を停止します..."
kill "$RECORD_PID" 2>/dev/null || true
wait "$RECORD_PID" 2>/dev/null || true

# 停止完了フラグを作成して nav-test に通知
touch "$DONE_FLAG"
echo "[rosbag-record] 記録を正常停止しました"
