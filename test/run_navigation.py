#!/usr/bin/env python3
"""
/navigate_to_pose アクションサーバーが起動するまで待機し、
ゴール到達後に終了するスクリプト。

使用方法:
  python3 run_navigation.py [goal_x] [goal_y] [--timeout TIMEOUT]

引数:
  goal_x   : ゴールのX座標 (デフォルト: 2.0)
  goal_y   : ゴールのY座標 (デフォルト: 0.5)
  --timeout: アクションサーバー待機・ナビゲーションのタイムアウト秒数 (デフォルト: 300)

終了コード:
  0: ゴール到達成功
  1: 失敗（タイムアウト、ゴール拒否、ナビゲーション失敗）
"""
import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationRunner(Node):
    def __init__(self):
        super().__init__('navigation_runner')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def set_initial_pose(self, x: float = 0.0, y: float = 0.0):
        """AMCLの初期位置を設定する"""
        pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        time.sleep(0.5)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.3)

        pub.destroy()
        self.get_logger().info(f'初期位置を設定しました: x={x}, y={y}')

    def run(self, goal_x: float, goal_y: float, timeout: float = 300.0) -> bool:
        """
        アクションサーバーが起動するまで待機してからゴールを送信し、
        到達結果を返す。

        Returns:
            True  : ゴール到達成功
            False : タイムアウト / 拒否 / 失敗
        """
        self.get_logger().info('/navigate_to_pose アクションサーバーを待機中...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f'アクションサーバーが {timeout}s 以内に起動しませんでした。'
            )
            return False

        self.get_logger().info('アクションサーバー接続完了。初期位置を設定します...')
        self.set_initial_pose()

        self.get_logger().info(f'ゴールを送信します: x={goal_x}, y={goal_y}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('ゴールが拒否されました。')
            return False

        self.get_logger().info('ゴール受理。到達を待機中...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        if not result_future.done():
            self.get_logger().error(f'ナビゲーションが {timeout}s 以内に完了しませんでした。')
            return False

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('ゴール到達成功！')
            return True
        else:
            self.get_logger().error(f'ナビゲーション失敗。ステータス: {result.status}')
            return False


def parse_args():
    parser = argparse.ArgumentParser(description='navigate_to_pose ゴール送信スクリプト')
    parser.add_argument('goal_x', type=float, nargs='?', default=2.0, help='ゴールX座標')
    parser.add_argument('goal_y', type=float, nargs='?', default=0.5, help='ゴールY座標')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='サーバー待機・ナビゲーションのタイムアウト秒数')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    runner = NavigationRunner()
    try:
        success = runner.run(
            goal_x=args.goal_x,
            goal_y=args.goal_y,
            timeout=args.timeout,
        )
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
