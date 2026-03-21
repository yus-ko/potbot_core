#!/bin/bash
# Gazebo + Navigation2 E2E テスト実行スクリプト
# 前提: Gazebo + Navigation2 が起動済みであること

set -e

echo "=== Gazebo + Navigation2 E2E テスト ==="
echo "前提: Gazebo と Navigation2 が起動済みであること"
echo ""

# Navigate to pose をCLIで実行
echo "ゴールポーズを送信: x=2.0, y=0.5"
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" && echo "ナビゲーション成功!" || echo "ナビゲーション失敗または タイムアウト"
