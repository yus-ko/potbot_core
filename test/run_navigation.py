#!/usr/bin/env python3
"""
/navigate_to_pose アクションサーバーが起動するまで待機し、
ゴール到達後に終了するスクリプト。

初期位置・ゴール位置・タイムアウトは waffle_pi.yaml の
navigation_runner.ros__parameters で一元管理する。

使用方法:
  python3 run_navigation.py --ros-args --params-file /path/to/waffle_pi.yaml

パラメーター（waffle_pi.yaml の navigation_runner.ros__parameters）:
  initial_pose_x : 初期位置X座標 (デフォルト: -2.0)
  initial_pose_y : 初期位置Y座標 (デフォルト: -0.5)
  goal_x         : ゴールX座標   (デフォルト:  2.0)
  goal_y         : ゴールY座標   (デフォルト:  0.5)
  timeout        : タイムアウト秒数 (デフォルト: 300.0)

終了コード:
  0: ゴール到達成功
  1: 失敗（タイムアウト、ゴール拒否、ナビゲーション失敗）
"""
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationRunner(Node):
    def __init__(self):
        super().__init__('navigation_runner')
        self.declare_parameter('initial_pose_x', -2.0)
        self.declare_parameter('initial_pose_y', -0.5)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('timeout', 300.0)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def set_initial_pose(self, x: float = 0.0, y: float = 0.0):
        """AMCLの初期位置を設定する。

        Publisher は明示的に破棄しない。
        pub.destroy() を呼ぶと QoS イベントが無効化され、その後の spin_once で
        InvalidHandle が発生する rclpy Humble の既知バグを回避するため。
        """
        if not hasattr(self, '_initialpose_pub'):
            self._initialpose_pub = self.create_publisher(
                PoseWithCovarianceStamped, 'initialpose', 10)
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
            self._initialpose_pub.publish(msg)
            time.sleep(0.3)

        self.get_logger().info(f'初期位置を設定しました: x={x}, y={y}')

    def run(self) -> bool:
        """
        ROSパラメーターからゴール・初期位置・タイムアウトを取得し、
        アクションサーバーが起動するまで待機してからゴールを送信する。

        Returns:
            True  : ゴール到達成功
            False : タイムアウト / 拒否 / 失敗
        """
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        timeout = self.get_parameter('timeout').value
        initial_pose_x = self.get_parameter('initial_pose_x').value
        initial_pose_y = self.get_parameter('initial_pose_y').value

        self.get_logger().info('/navigate_to_pose アクションサーバーを待機中...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f'アクションサーバーが {timeout}s 以内に起動しませんでした。'
            )
            return False

        self.get_logger().info('アクションサーバー接続完了。初期位置を設定します...')
        self.set_initial_pose(initial_pose_x, initial_pose_y)

        # wait_for_server は Action Server の存在のみを確認するため、
        # bt_navigator の lifecycle 状態が active になるまで待機する
        self.get_logger().info('nav2 の完全な起動を待機中 (bt_navigator active 検知)...')
        if not self._wait_for_nav2_active(timeout=timeout):
            self.get_logger().error('nav2 が active になりませんでした。')
            return False

        # ゴール送信（nav2 が拒否した場合はリトライ）
        MAX_RETRIES = 5
        RETRY_INTERVAL = 5.0

        for attempt in range(1, MAX_RETRIES + 1):
            self.get_logger().info(
                f'ゴールを送信します (試行 {attempt}/{MAX_RETRIES}): x={goal_x}, y={goal_y}'
            )
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.orientation.w = 1.0

            send_future = self._client.send_goal_async(goal_msg)
            if not self._spin_until_done(send_future, timeout_sec=15.0):
                self.get_logger().warn(f'ゴール送信がタイムアウト (試行 {attempt})。リトライします...')
                time.sleep(RETRY_INTERVAL)
                continue

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn(f'ゴールが拒否されました (試行 {attempt})。リトライします...')
                time.sleep(RETRY_INTERVAL)
                continue

            # ゴール受理 → 結果待ち
            self.get_logger().info('ゴール受理。到達を待機中...')
            result_future = goal_handle.get_result_async()
            if not self._spin_until_done(result_future, timeout_sec=timeout):
                self.get_logger().error(f'ナビゲーションが {timeout}s 以内に完了しませんでした。')
                return False

            result = result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('ゴール到達成功！')
                return True
            else:
                self.get_logger().error(f'ナビゲーション失敗。ステータス: {result.status}')
                return False

        self.get_logger().error(f'{MAX_RETRIES} 回試行しましたがゴールを送信できませんでした。')
        return False

    def _wait_for_nav2_active(self, timeout: float = 60.0) -> bool:
        """bt_navigator の lifecycle 状態が active (id=3) になるまで待機する。

        wait_for_server() は Action Server の存在のみを確認するため、
        nav2 の全ノードが active になるまでの時間差を lifecycle サービスで検知する。

        Returns:
            True : active 状態を確認できた
            False: timeout 以内に active にならなかった
        """
        # ACTIVE 状態の ID (lifecycle_msgs/msg/State)
        ACTIVE_STATE_ID = 3

        client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                if not client.service_is_ready():
                    time.sleep(0.5)
                    continue
                future = client.call_async(GetState.Request())
                if not self._spin_until_done(future, timeout_sec=5.0):
                    time.sleep(1.0)
                    continue
                state_id = future.result().current_state.id
                if state_id == ACTIVE_STATE_ID:
                    self.get_logger().info('bt_navigator が active 状態になりました。')
                    return True
                self.get_logger().debug(f'bt_navigator 状態: {state_id} (active 待ち...)')
                time.sleep(1.0)
        finally:
            client.destroy()
        return False

    def _spin_until_done(self, future, timeout_sec: float) -> bool:
        """future が完了するまで spin_once ループで待機する。

        rclpy Humble には qos_event の InvalidHandle バグがあるため、
        例外を無視しながら spin し続ける。

        Returns:
            True : future が完了した
            False: timeout_sec 以内に完了しなかった
        """
        import time as _time
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        deadline = _time.monotonic() + timeout_sec
        try:
            while not future.done():
                remaining = deadline - _time.monotonic()
                if remaining <= 0:
                    return False
                try:
                    executor.spin_once(timeout_sec=min(0.1, remaining))
                except Exception:
                    # rclpy Humble の QoS InvalidHandle バグを無視して継続
                    pass
        finally:
            executor.remove_node(self)
            executor.shutdown()
        return True


def main():
    rclpy.init()
    runner = NavigationRunner()
    try:
        success = runner.run()
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
