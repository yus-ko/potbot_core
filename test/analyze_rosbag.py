#!/usr/bin/env python3
"""rosbag2 データを解析し、ロボットの軌跡・速度指令・計画経路を可視化するスクリプト。

rosbags ライブラリを使用して rosbag2 (sqlite3形式) を読み込み、
/odom、/cmd_vel、/plan トピックからデータを抽出して matplotlib で4パネルの
図を生成・保存する。
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


def parse_args():
    """コマンドライン引数を解析する。"""
    parser = argparse.ArgumentParser(
        description='rosbag2 データを解析し、軌跡・速度・計画経路を可視化する'
    )
    parser.add_argument(
        '--bag-path',
        type=str,
        required=True,
        help='rosbag2 ディレクトリのパス',
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='出力PNG保存先ディレクトリ (デフォルト: bag-pathの親ディレクトリ)',
    )
    parser.add_argument(
        '--goal-x',
        type=float,
        default=-1.25,
        help='ゴール地点のX座標 [m] (デフォルト: -1.25)',
    )
    parser.add_argument(
        '--goal-y',
        type=float,
        default=3.5,
        help='ゴール地点のY座標 [m] (デフォルト: 3.5)',
    )
    return parser.parse_args()


def read_rosbag(bag_path):
    """rosbag2 からトピックデータを読み込む。

    Args:
        bag_path: rosbag2 ディレクトリのパス。

    Returns:
        odom_data: (timestamps, xs, ys) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。
        plan_data: (plan_timestamps, plan_paths) のタプル。
            plan_timestamps: 各 /plan メッセージのタイムスタンプリスト [s]。
            plan_paths: 各 /plan メッセージの [(x, y), ...] リスト。
    """
    odom_timestamps = []
    odom_xs = []
    odom_ys = []

    cmd_vel_timestamps = []
    cmd_vel_linear_xs = []
    cmd_vel_angular_zs = []

    plan_timestamps = []
    plan_paths = []

    start_time = None
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if start_time is None:
                start_time = timestamp

            time_sec = (timestamp - start_time) / 1e9

            if connection.topic == '/odom':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                odom_timestamps.append(time_sec)
                odom_xs.append(msg.pose.pose.position.x)
                odom_ys.append(msg.pose.pose.position.y)

            elif connection.topic == '/cmd_vel':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                cmd_vel_timestamps.append(time_sec)
                cmd_vel_linear_xs.append(msg.linear.x)
                cmd_vel_angular_zs.append(msg.angular.z)

            elif connection.topic == '/plan':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                path_points = [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in msg.poses
                ]
                plan_timestamps.append(time_sec)
                plan_paths.append(path_points)

    odom_data = (odom_timestamps, odom_xs, odom_ys)
    cmd_vel_data = (cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs)
    plan_data = (plan_timestamps, plan_paths)
    return odom_data, cmd_vel_data, plan_data


def create_figure(odom_data, cmd_vel_data, plan_data, goal_x=-1.25, goal_y=3.5):
    """4パネルの図を生成する。

    Args:
        odom_data: (timestamps, xs, ys) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。
        plan_data: (plan_timestamps, plan_paths) のタプル。
        goal_x: ゴール地点のX座標 [m]。
        goal_y: ゴール地点のY座標 [m]。

    Returns:
        matplotlib の Figure オブジェクト。
    """
    odom_timestamps, odom_xs, odom_ys = odom_data
    cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs = cmd_vel_data
    plan_timestamps, plan_paths = plan_data

    fig, axes = plt.subplots(4, 1, figsize=(10, 16), constrained_layout=True)

    ax_xy = axes[0]

    for path in plan_paths[:-1]:
        if path:
            xs, ys = zip(*path)
            ax_xy.plot(xs, ys, color='gray', linewidth=0.8, alpha=0.5)

    if plan_paths:
        latest_path = plan_paths[-1]
        if latest_path:
            xs, ys = zip(*latest_path)
            ax_xy.plot(xs, ys, 'r-', linewidth=1.5, label='Plan (latest)')

    if len(plan_paths) > 1:
        ax_xy.plot([], [], color='gray', linewidth=0.8, alpha=0.5, label='Plan (old)')

    ax_xy.plot(odom_xs, odom_ys, 'b-', label='Trajectory (odom)')
    if odom_xs:
        ax_xy.plot(odom_xs[0], odom_ys[0], 'go', markersize=10, label='Start')

    ax_xy.plot(goal_x, goal_y, 'r^', markersize=10, label='Goal')
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_aspect('equal')
    ax_xy.grid(True)
    ax_xy.set_title('Robot Trajectory')
    ax_xy.legend()

    ax_lin = axes[1]
    ax_lin.plot(cmd_vel_timestamps, cmd_vel_linear_xs, 'b-')
    ax_lin.set_xlabel('Time [s]')
    ax_lin.set_ylabel('Linear Velocity [m/s]')
    ax_lin.grid(True)
    ax_lin.set_title('Linear Velocity (cmd_vel)')

    ax_ang = axes[2]
    ax_ang.plot(cmd_vel_timestamps, cmd_vel_angular_zs, 'b-')
    ax_ang.set_xlabel('Time [s]')
    ax_ang.set_ylabel('Angular Velocity [rad/s]')
    ax_ang.grid(True)
    ax_ang.set_title('Angular Velocity (cmd_vel)')

    ax_plan = axes[3]
    if plan_timestamps and plan_paths:
        points = [
            (x, y, t)
            for t, path in zip(plan_timestamps, plan_paths)
            for x, y in path
        ]
        scatter_xs, scatter_ys, scatter_times = zip(*points)
        sc = ax_plan.scatter(
            scatter_xs, scatter_ys,
            c=scatter_times,
            cmap='viridis',
            s=2,
            alpha=0.6,
        )
        fig.colorbar(sc, ax=ax_plan, label='Time [s]')
    ax_plan.set_xlabel('Plan Path X [m]')
    ax_plan.set_ylabel('Plan Path Y [m]')
    ax_plan.set_aspect('equal')
    ax_plan.grid(True)
    ax_plan.set_title('Planned Path Points Over Time')

    return fig


def main():
    """メイン処理。"""
    args = parse_args()

    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f'エラー: rosbag2 ディレクトリが見つかりません: {bag_path}',
              file=sys.stderr)
        sys.exit(1)

    output_dir = Path(args.output_dir) if args.output_dir else bag_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    odom_data, cmd_vel_data, plan_data = read_rosbag(str(bag_path))

    fig = create_figure(odom_data, cmd_vel_data, plan_data,
                        goal_x=args.goal_x, goal_y=args.goal_y)

    output_path = output_dir / 'navigation_result.png'
    fig.savefig(str(output_path), dpi=150)
    plt.close(fig)

    print(output_path)


if __name__ == '__main__':
    main()
