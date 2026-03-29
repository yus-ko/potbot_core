#!/usr/bin/env python3
"""ランダムゴールナビゲーションテスト

マップPGMから走行可能領域を解析し、ランダムなゴールを逐次送信して
スタック・停留を検出するスクリプト。

使用方法:
  python3 random_navigation.py --ros-args --params-file /path/to/waffle_pi.yaml

パラメーター（waffle_pi.yaml の random_navigation_runner.ros__parameters）:
  initial_pose_x/y : 初期位置（AMCL設定と一致させること）
  map_yaml         : マップYAMLファイルパス
  num_goals        : ゴール数（0=無限ループ）
  expected_speed   : 想定巡航速度 [m/s]
  safety_margin    : タイムアウト安全係数
  min_timeout      : 最小タイムアウト [s]
  stuck_check_interval : 停留チェック間隔 [s]
  stuck_distance_threshold : 停留判定の移動距離閾値 [m]
  max_stuck_count  : 停留連続回数でゴールキャンセル
  summary_interval : サマリ出力間隔（ゴール数）
  results_dir      : 結果出力ディレクトリ
  goal_region_x_min/max/y_min/max : ゴール生成矩形範囲（オプション）

終了コード:
  0: 全ゴール完了
  1: 致命的エラー（Nav2未起動等）
"""

import csv
import json
import math
import os
import random
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from PIL import Image
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.ndimage import label


class MapAnalyzer:
    """マップPGM + YAMLから走行可能領域を解析し、ランダムゴールを生成する。"""

    def __init__(self, map_yaml_path: str):
        with open(map_yaml_path) as f:
            map_config = yaml.safe_load(f)

        map_dir = Path(map_yaml_path).parent
        pgm_path = map_dir / map_config['image']

        self.resolution = map_config['resolution']
        self.origin_x = map_config['origin'][0]
        self.origin_y = map_config['origin'][1]
        self.free_thresh = map_config.get('free_thresh', 0.196)
        self.occupied_thresh = map_config.get('occupied_thresh', 0.65)
        negate = map_config.get('negate', 0)

        img = Image.open(str(pgm_path)).convert('L')
        grid = np.array(img, dtype=np.float64) / 255.0

        if negate:
            grid = 1.0 - grid

        # free cell: 値が (1 - free_thresh) より大きい（PGMでは白=free）
        self.free_mask = grid > (1.0 - self.free_thresh)
        # occupied cell: 値が (1 - occupied_thresh) より小さい（PGMでは黒=occupied）
        self.occupied_mask = grid < (1.0 - self.occupied_thresh)
        self.height, self.width = self.free_mask.shape

    def get_reachable_cells(self, robot_x: float, robot_y: float):
        """ロボット位置を含む連結成分のfree cellインデックスを返す。"""
        labeled, num_features = label(self.free_mask)

        # ワールド座標→ピクセル座標
        px = int((robot_x - self.origin_x) / self.resolution)
        # PGMは上が0行目だが、originは左下なのでY軸を反転
        py = self.height - 1 - int((robot_y - self.origin_y) / self.resolution)

        px = max(0, min(px, self.width - 1))
        py = max(0, min(py, self.height - 1))

        robot_label = labeled[py, px]
        if robot_label == 0:
            # ロボット位置がfree cellでない場合、最近傍のfree cellを探す
            free_ys, free_xs = np.where(self.free_mask)
            if len(free_ys) == 0:
                return np.array([]), np.array([])
            dists = (free_xs - px) ** 2 + (free_ys - py) ** 2
            nearest = np.argmin(dists)
            robot_label = labeled[free_ys[nearest], free_xs[nearest]]

        component_mask = labeled == robot_label
        return np.where(component_mask)

    def generate_random_goals(self, robot_x: float, robot_y: float, n: int,
                              goal_region=None, seed=None,
                              min_distance: float = 2.0):
        """到達可能領域からランダムにn個のワールド座標ゴールを生成する。

        Args:
            robot_x, robot_y: ロボット初期位置 [m]
            n: 生成するゴール数
            goal_region: {'x_min','x_max','y_min','y_max'} の辞書（オプション）
            seed: 乱数シード（再現性のため）
            min_distance: ロボットからの最小距離 [m]（近すぎるゴールを除外）

        Returns:
            list of (x, y) タプル
        """
        if seed is not None:
            random.seed(seed)

        rows, cols = self.get_reachable_cells(robot_x, robot_y)
        if len(rows) == 0:
            return []

        # ピクセル座標→ワールド座標の候補リスト
        world_coords = []
        for r, c in zip(rows, cols):
            wx = self.origin_x + c * self.resolution
            wy = self.origin_y + (self.height - 1 - r) * self.resolution
            if goal_region:
                if not (goal_region['x_min'] <= wx <= goal_region['x_max'] and
                        goal_region['y_min'] <= wy <= goal_region['y_max']):
                    continue
            # 最小距離フィルタ
            dist = math.sqrt((wx - robot_x) ** 2 + (wy - robot_y) ** 2)
            if dist < min_distance:
                continue
            world_coords.append((wx, wy))

        if not world_coords:
            return []

        goals = []
        for _ in range(n):
            goals.append(random.choice(world_coords))
        return goals

    def is_occupied(self, wx: float, wy: float) -> bool:
        """ワールド座標がoccupiedセルかどうかを判定する。"""
        px = int((wx - self.origin_x) / self.resolution)
        py = self.height - 1 - int((wy - self.origin_y) / self.resolution)
        if px < 0 or px >= self.width or py < 0 or py >= self.height:
            return False
        return bool(self.occupied_mask[py, px])

    def check_path_wall_penetration(self, path_points, interpolation_step=0.05):
        """経路点と内挿点が壁を貫通していないかチェックする。

        Args:
            path_points: [(x, y), ...] の経路点リスト
            interpolation_step: 内挿間隔 [m]

        Returns:
            list of (x, y): 壁貫通が検出された座標のリスト（空なら貫通なし）
        """
        violations = []
        for i in range(len(path_points)):
            x, y = path_points[i]
            if self.is_occupied(x, y):
                violations.append((x, y))

            # 次の点との間を内挿
            if i + 1 < len(path_points):
                nx, ny = path_points[i + 1]
                dx = nx - x
                dy = ny - y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < 1e-6:
                    continue
                n_steps = max(1, int(dist / interpolation_step))
                for s in range(1, n_steps):
                    t = s / n_steps
                    ix = x + dx * t
                    iy = y + dy * t
                    if self.is_occupied(ix, iy):
                        violations.append((ix, iy))
        return violations


class RandomNavigationRunner(Node):
    """ランダムゴールを逐次送信し、停留検出・結果記録を行うノード。"""

    def __init__(self):
        super().__init__('random_navigation_runner')

        # パラメータ宣言
        self.declare_parameter('initial_pose_x', -2.0)
        self.declare_parameter('initial_pose_y', -0.5)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('num_goals', 100)
        self.declare_parameter('expected_speed', 0.2)
        self.declare_parameter('safety_margin', 3.0)
        self.declare_parameter('min_timeout', 30.0)
        self.declare_parameter('stuck_check_interval', 10.0)
        self.declare_parameter('stuck_distance_threshold', 0.1)
        self.declare_parameter('max_stuck_count', 3)
        self.declare_parameter('summary_interval', 10)
        self.declare_parameter('results_dir', '/root/test/results')
        self.declare_parameter('goal_region_x_min', float('nan'))
        self.declare_parameter('goal_region_x_max', float('nan'))
        self.declare_parameter('goal_region_y_min', float('nan'))
        self.declare_parameter('goal_region_y_max', float('nan'))
        self.declare_parameter('timeout', 300.0)
        self.declare_parameter('seed', -1)
        # 固定ゴール指定（NaN以外の場合はランダム生成を無視してこのゴールを使用）
        self.declare_parameter('fixed_goal_x', float('nan'))
        self.declare_parameter('fixed_goal_y', float('nan'))

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_pub = self.create_publisher(PoseStamped, '/test/goal_pose', 10)

        # odom購読（停留検出用）
        self._current_x = 0.0
        self._current_y = 0.0
        self._odom_received = False
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # /plan購読（壁貫通検出用）
        self._latest_plan = []
        self._plan_wall_violations = 0
        self._plan_sub = self.create_subscription(
            NavPath, '/plan', self._plan_callback, 10)
        self._map_analyzer = None  # run()で設定

        # シャットダウンフラグ
        self._shutdown = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # 結果リスト
        self._results = []

    def _signal_handler(self, signum, frame):
        self.get_logger().info('シャットダウンシグナルを受信しました。現在のゴール完了後に終了します。')
        self._shutdown = True

    def _odom_callback(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._odom_received = True

    def _plan_callback(self, msg):
        """グローバルパスを受信し、壁貫通チェックを行う。"""
        path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self._latest_plan = path_points
        if self._map_analyzer and path_points:
            violations = self._map_analyzer.check_path_wall_penetration(path_points)
            if violations:
                self._plan_wall_violations += len(violations)
                self.get_logger().error(
                    f'壁貫通検出！ {len(violations)}点が障害物セル上: '
                    f'例=({violations[0][0]:.2f}, {violations[0][1]:.2f})')

    def set_initial_pose(self, x: float, y: float):
        """AMCLの初期位置を設定する。"""
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

    def _wait_for_nav2_active(self, timeout: float = 60.0) -> bool:
        """bt_navigator の lifecycle 状態が active になるまで待機する。"""
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
        """future が完了するまで spin_once ループで待機する。"""
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        deadline = time.monotonic() + timeout_sec
        try:
            while not future.done():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                try:
                    executor.spin_once(timeout_sec=min(0.1, remaining))
                except Exception:
                    pass
        finally:
            executor.remove_node(self)
            executor.shutdown()
        return True

    def _compute_timeout(self, distance: float) -> float:
        """距離に応じたタイムアウトを計算する。"""
        expected_speed = self.get_parameter('expected_speed').value
        safety_margin = self.get_parameter('safety_margin').value
        min_timeout = self.get_parameter('min_timeout').value
        return max(min_timeout, distance / expected_speed * safety_margin)

    def _navigate_to_goal(self, goal_x: float, goal_y: float, timeout: float):
        """単一ゴールへのナビゲーションを実行し、停留を監視する。

        Returns:
            dict: {result, duration, stuck_count, path_length}
        """
        stuck_check_interval = self.get_parameter('stuck_check_interval').value
        stuck_dist_thresh = self.get_parameter('stuck_distance_threshold').value
        max_stuck = self.get_parameter('max_stuck_count').value

        # ゴールを /test/goal_pose にパブリッシュ（rosbag記録用）
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_x
        goal_pose_msg.pose.position.y = goal_y
        goal_pose_msg.pose.orientation.w = 1.0
        self._goal_pub.publish(goal_pose_msg)

        # ゴール送信
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        MAX_RETRIES = 5
        RETRY_INTERVAL = 5.0
        goal_handle = None

        for attempt in range(1, MAX_RETRIES + 1):
            self.get_logger().info(
                f'ゴール送信 (試行 {attempt}/{MAX_RETRIES}): x={goal_x:.2f}, y={goal_y:.2f}')
            send_future = self._client.send_goal_async(goal_msg)
            if not self._spin_until_done(send_future, timeout_sec=15.0):
                self.get_logger().warn(f'ゴール送信タイムアウト (試行 {attempt})')
                time.sleep(RETRY_INTERVAL)
                continue
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            self.get_logger().warn(f'ゴール拒否 (試行 {attempt})')
            time.sleep(RETRY_INTERVAL)
            goal_handle = None

        if goal_handle is None:
            return {'result': 'rejected', 'duration': 0.0, 'stuck_count': 0, 'path_length': 0.0}

        # 結果待ち + 停留検出ループ
        self.get_logger().info('ゴール受理。ナビゲーション中...')
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        last_check_time = start_time
        last_check_x = self._current_x
        last_check_y = self._current_y
        stuck_count = 0
        path_length = 0.0
        prev_x = self._current_x
        prev_y = self._current_y

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        try:
            while not result_future.done():
                elapsed = time.monotonic() - start_time
                if elapsed > timeout:
                    self.get_logger().warn(f'タイムアウト ({timeout:.0f}s)')
                    goal_handle.cancel_goal_async()
                    # キャンセル完了を少し待つ
                    self._spin_brief(executor, 2.0)
                    return {
                        'result': 'timeout',
                        'duration': elapsed,
                        'stuck_count': stuck_count,
                        'path_length': path_length,
                    }

                try:
                    executor.spin_once(timeout_sec=0.1)
                except Exception:
                    pass

                # 経路長の累積
                dx = self._current_x - prev_x
                dy = self._current_y - prev_y
                path_length += math.sqrt(dx * dx + dy * dy)
                prev_x = self._current_x
                prev_y = self._current_y

                # 停留チェック
                now = time.monotonic()
                if now - last_check_time >= stuck_check_interval:
                    move_dist = math.sqrt(
                        (self._current_x - last_check_x) ** 2 +
                        (self._current_y - last_check_y) ** 2)
                    if move_dist < stuck_dist_thresh:
                        stuck_count += 1
                        self.get_logger().warn(
                            f'停留検出 ({stuck_count}/{max_stuck}): '
                            f'移動距離={move_dist:.3f}m < {stuck_dist_thresh}m')
                        if stuck_count >= max_stuck:
                            self.get_logger().error(
                                f'連続{max_stuck}回停留。ゴールをキャンセルします。')
                            goal_handle.cancel_goal_async()
                            self._spin_brief(executor, 2.0)
                            return {
                                'result': 'stuck',
                                'duration': time.monotonic() - start_time,
                                'stuck_count': stuck_count,
                                'path_length': path_length,
                            }
                    else:
                        stuck_count = 0
                    last_check_x = self._current_x
                    last_check_y = self._current_y
                    last_check_time = now
        finally:
            executor.remove_node(self)
            executor.shutdown()

        duration = time.monotonic() - start_time
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return {
                'result': 'success',
                'duration': duration,
                'stuck_count': stuck_count,
                'path_length': path_length,
            }
        else:
            return {
                'result': f'failed(status={result.status})',
                'duration': duration,
                'stuck_count': stuck_count,
                'path_length': path_length,
            }

    def _spin_brief(self, executor, seconds):
        """短時間spinしてコールバックを処理する。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass

    def _write_csv_row(self, csv_path, row):
        """CSV に1行追記する。ファイルが存在しなければヘッダ付きで作成。"""
        file_exists = os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'goal_id', 'goal_x', 'goal_y', 'start_x', 'start_y',
                'distance', 'result', 'duration_sec', 'stuck_count',
                'path_length', 'timeout_sec', 'wall_violations',
            ])
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def _write_summary(self, results_dir):
        """サマリを stdout + summary_latest.json に出力する。"""
        total = len(self._results)
        if total == 0:
            return

        success = sum(1 for r in self._results if r['result'] == 'success')
        stuck = sum(1 for r in self._results if r['result'] == 'stuck')
        timeout = sum(1 for r in self._results if r['result'] == 'timeout')
        rejected = sum(1 for r in self._results if r['result'] == 'rejected')
        failed = total - success - stuck - timeout - rejected

        durations = [r['duration_sec'] for r in self._results if r['result'] == 'success']
        avg_duration = sum(durations) / len(durations) if durations else 0.0

        summary = {
            'total_goals': total,
            'success': success,
            'stuck': stuck,
            'timeout': timeout,
            'rejected': rejected,
            'failed': failed,
            'success_rate': success / total if total > 0 else 0.0,
            'stuck_rate': stuck / total if total > 0 else 0.0,
            'avg_success_duration_sec': round(avg_duration, 2),
        }

        summary_str = (
            f"\n{'='*50}\n"
            f"  サマリ ({total} ゴール完了)\n"
            f"{'='*50}\n"
            f"  成功: {success} ({summary['success_rate']*100:.1f}%)\n"
            f"  スタック: {stuck} ({summary['stuck_rate']*100:.1f}%)\n"
            f"  タイムアウト: {timeout}\n"
            f"  拒否: {rejected}\n"
            f"  その他失敗: {failed}\n"
            f"  平均成功所要時間: {avg_duration:.1f}s\n"
            f"{'='*50}\n"
        )
        self.get_logger().info(summary_str)

        summary_path = os.path.join(results_dir, 'summary_latest.json')
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

    def run(self) -> bool:
        """メイン実行ループ。"""
        initial_x = self.get_parameter('initial_pose_x').value
        initial_y = self.get_parameter('initial_pose_y').value
        map_yaml = self.get_parameter('map_yaml').value
        num_goals = self.get_parameter('num_goals').value
        summary_interval = self.get_parameter('summary_interval').value
        results_dir = self.get_parameter('results_dir').value
        timeout = self.get_parameter('timeout').value
        seed_val = self.get_parameter('seed').value

        if not map_yaml:
            self.get_logger().error('map_yaml パラメータが設定されていません。')
            return False

        # ゴール領域（オプション）
        goal_region = None
        xmin = self.get_parameter('goal_region_x_min').value
        xmax = self.get_parameter('goal_region_x_max').value
        ymin = self.get_parameter('goal_region_y_min').value
        ymax = self.get_parameter('goal_region_y_max').value
        if not (math.isnan(xmin) or math.isnan(xmax) or
                math.isnan(ymin) or math.isnan(ymax)):
            goal_region = {
                'x_min': xmin, 'x_max': xmax,
                'y_min': ymin, 'y_max': ymax,
            }
            self.get_logger().info(
                f'ゴール領域: x=[{xmin}, {xmax}], y=[{ymin}, {ymax}]')

        # マップ解析
        self.get_logger().info(f'マップを解析中: {map_yaml}')
        analyzer = MapAnalyzer(map_yaml)
        self._map_analyzer = analyzer  # 壁貫通チェック用

        # ゴール生成設定
        seed = seed_val if seed_val >= 0 else None
        if seed is not None:
            random.seed(seed)
        infinite_mode = (num_goals == 0)
        if infinite_mode:
            self.get_logger().info('無限ループモードで実行します。Ctrl+C で停止。')

        # 初期位置から到達可能か事前チェック
        test_goals = analyzer.generate_random_goals(
            initial_x, initial_y, 1, goal_region=goal_region)
        if not test_goals:
            self.get_logger().error('到達可能なゴールを生成できませんでした。')
            return False
        self.get_logger().info('マップ解析完了。ゴールは各ナビゲーション直前に現在位置基準で生成します。')

        # Nav2 起動待ち
        self.get_logger().info('/navigate_to_pose アクションサーバーを待機中...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('アクションサーバーが起動しませんでした。')
            return False

        self.set_initial_pose(initial_x, initial_y)

        self.get_logger().info('nav2 の完全な起動を待機中...')
        if not self._wait_for_nav2_active(timeout=timeout):
            self.get_logger().error('nav2 が active になりませんでした。')
            return False

        # odom受信待ち
        self.get_logger().info('odom データを待機中...')
        wait_start = time.monotonic()
        while not self._odom_received:
            if time.monotonic() - wait_start > 30.0:
                self.get_logger().error('odom データを受信できませんでした。')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        # CSV パス
        csv_path = os.path.join(results_dir, 'navigation_results.csv')

        # 固定ゴール指定チェック
        fixed_gx = self.get_parameter('fixed_goal_x').value
        fixed_gy = self.get_parameter('fixed_goal_y').value
        use_fixed_goal = not (math.isnan(fixed_gx) or math.isnan(fixed_gy))
        if use_fixed_goal:
            self.get_logger().info(
                f'固定ゴールモード: ({fixed_gx:.2f}, {fixed_gy:.2f})')

        # ナビゲーションループ
        goal_id = 0
        while True:
            if self._shutdown:
                self.get_logger().info('シャットダウン要求を受けました。終了します。')
                break

            if not infinite_mode and goal_id >= num_goals:
                break

            if use_fixed_goal:
                gx, gy = fixed_gx, fixed_gy
            else:
                # 現在位置基準でゴールを1つ生成
                current_goals = analyzer.generate_random_goals(
                    self._current_x, self._current_y, 1,
                    goal_region=goal_region)
                if not current_goals:
                    self.get_logger().warn(
                        f'現在位置({self._current_x:.1f}, {self._current_y:.1f})から'
                        f'到達可能なゴールなし。初期位置基準にフォールバック。')
                    current_goals = analyzer.generate_random_goals(
                        initial_x, initial_y, 1, goal_region=goal_region)
                    if not current_goals:
                        self.get_logger().error('ゴールを生成できません。終了します。')
                        break
                gx, gy = current_goals[0]

            goal_id += 1

            start_x = self._current_x
            start_y = self._current_y
            distance = math.sqrt((gx - start_x) ** 2 + (gy - start_y) ** 2)
            goal_timeout = self._compute_timeout(distance)

            self.get_logger().info(
                f'\n--- ゴール {goal_id}'
                f'{f"/{num_goals}" if not infinite_mode else ""} ---\n'
                f'  目標: ({gx:.2f}, {gy:.2f})\n'
                f'  出発: ({start_x:.2f}, {start_y:.2f})\n'
                f'  距離: {distance:.2f}m\n'
                f'  タイムアウト: {goal_timeout:.0f}s')

            self._plan_wall_violations = 0  # ゴールごとにリセット
            nav_result = self._navigate_to_goal(gx, gy, goal_timeout)

            row = {
                'goal_id': goal_id,
                'goal_x': round(gx, 3),
                'goal_y': round(gy, 3),
                'start_x': round(start_x, 3),
                'start_y': round(start_y, 3),
                'distance': round(distance, 3),
                'result': nav_result['result'],
                'duration_sec': round(nav_result['duration'], 2),
                'stuck_count': nav_result['stuck_count'],
                'path_length': round(nav_result['path_length'], 3),
                'timeout_sec': round(goal_timeout, 1),
                'wall_violations': self._plan_wall_violations,
            }
            self._results.append(row)
            self._write_csv_row(csv_path, row)

            status_emoji = {
                'success': 'OK', 'stuck': 'STUCK',
                'timeout': 'TIMEOUT', 'rejected': 'REJECTED',
            }.get(nav_result['result'], 'FAIL')
            wall_str = f', 壁貫通={self._plan_wall_violations}' if self._plan_wall_violations > 0 else ''
            self.get_logger().info(
                f'  結果: [{status_emoji}] {nav_result["result"]} '
                f'({nav_result["duration"]:.1f}s, '
                f'停留={nav_result["stuck_count"]}, '
                f'走行={nav_result["path_length"]:.1f}m{wall_str})')

            # 定期サマリ
            if goal_id % summary_interval == 0:
                self._write_summary(results_dir)

        # 最終サマリ
        self._write_summary(results_dir)
        return True


def main():
    rclpy.init()
    runner = RandomNavigationRunner()
    try:
        success = runner.run()
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
