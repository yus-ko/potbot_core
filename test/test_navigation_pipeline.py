"""
Gazebo + Navigation2 E2Eテストパイプライン
ゴールポーズ発行からゴール到着までの一連の誘導を検証する
"""
import pytest
import rclpy
import unittest
import time
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

import launch
import launch_ros
import launch_testing
import launch_testing.actions

# テスト設定
GOAL_X = 2.0
GOAL_Y = 0.5
GOAL_TOLERANCE = 0.3  # meters
NAVIGATION_TIMEOUT = 120.0  # seconds
NAV2_STARTUP_TIMEOUT = 60.0  # seconds

@pytest.mark.launch_test
def generate_test_description():
    """テスト起動記述を生成 - Navigation2が事前起動済みを想定"""
    # このテストは外部でGazebo + Nav2が起動済みの環境で実行
    # launch_testing ではダミーのローンチを提供
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])

class TestNavigationPipeline(unittest.TestCase):
    """ナビゲーションパイプラインのE2Eテスト"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_e2e_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_navigate_to_pose_action_available(self):
        """navigate_to_poseアクションサーバーが利用可能か確認"""
        from rclpy.action import ActionClient
        client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        available = client.wait_for_server(timeout_sec=NAV2_STARTUP_TIMEOUT)
        self.assertTrue(available, "navigate_to_pose action server not available")
        client.destroy()

    def test_full_navigation_cycle(self):
        """ゴールポーズ発行からゴール到着までの一連の誘導テスト"""
        from rclpy.action import ActionClient

        # AMCL初期位置の設定
        initialpose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        time.sleep(1.0)

        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.node.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.06853891945200942

        for _ in range(3):
            initialpose_pub.publish(init_pose)
            time.sleep(0.5)

        # ゴール送信
        client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(
            client.wait_for_server(timeout_sec=NAV2_STARTUP_TIMEOUT),
            "navigate_to_pose action server not available"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = GOAL_X
        goal_msg.pose.pose.position.y = GOAL_Y
        goal_msg.pose.pose.orientation.w = 1.0

        result_container = {'status': None, 'done': False}

        def result_callback(future):
            result = future.result()
            result_container['status'] = result.status
            result_container['done'] = True

        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        self.assertIsNotNone(goal_handle, "Goal was rejected")
        self.assertTrue(goal_handle.accepted, "Goal was rejected")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

        elapsed = 0.0
        while not result_container['done'] and elapsed < NAVIGATION_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            elapsed += 1.0

        self.assertTrue(
            result_container['done'],
            f"Navigation did not complete within {NAVIGATION_TIMEOUT}s"
        )
        self.assertEqual(
            result_container['status'],
            GoalStatus.STATUS_SUCCEEDED,
            f"Navigation failed with status: {result_container['status']}"
        )

        client.destroy()
        initialpose_pub.destroy()
