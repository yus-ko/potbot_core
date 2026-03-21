#!/usr/bin/env python3
"""rosbag2 データを解析し、ロボットの軌跡と速度指令を可視化するスクリプト。

rosbags ライブラリを使用して rosbag2 (sqlite3形式) を読み込み、
/odom と /cmd_vel トピックからデータを抽出して matplotlib で3パネルの
図を生成・保存する。
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def parse_args():
    """コマンドライン引数を解析する。"""
    parser = argparse.ArgumentParser(
        description='rosbag2 データを解析し、軌跡と速度を可視化する'
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
    return parser.parse_args()


def read_rosbag(bag_path):
    """rosbag2 からトピックデータを読み込む。

    Args:
        bag_path: rosbag2 ディレクトリのパス。

    Returns:
        odom_data: (timestamps, xs, ys) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。
    """
    odom_timestamps = []
    odom_xs = []
    odom_ys = []

    cmd_vel_timestamps = []
    cmd_vel_linear_xs = []
    cmd_vel_angular_zs = []

    start_time = None

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if start_time is None:
                start_time = timestamp

            time_sec = (timestamp - start_time) / 1e9

            if connection.topic == '/odom':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                odom_timestamps.append(time_sec)
                odom_xs.append(msg.pose.pose.position.x)
                odom_ys.append(msg.pose.pose.position.y)

            elif connection.topic == '/cmd_vel':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                cmd_vel_timestamps.append(time_sec)
                cmd_vel_linear_xs.append(msg.linear.x)
                cmd_vel_angular_zs.append(msg.angular.z)

    odom_data = (odom_timestamps, odom_xs, odom_ys)
    cmd_vel_data = (cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs)
    return odom_data, cmd_vel_data


def create_figure(odom_data, cmd_vel_data):
    """3パネルの図を生成する。

    Args:
        odom_data: (timestamps, xs, ys) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。

    Returns:
        matplotlib の Figure オブジェクト。
    """
    odom_timestamps, odom_xs, odom_ys = odom_data
    cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs = cmd_vel_data

    fig, axes = plt.subplots(3, 1, figsize=(10, 12))

    # パネル1: XY軌跡プロット
    ax_xy = axes[0]
    ax_xy.plot(odom_xs, odom_ys, 'b-', label='Trajectory')
    if odom_xs:
        ax_xy.plot(odom_xs[0], odom_ys[0], 'go', markersize=10, label='Start')
    ax_xy.plot(2.0, 0.5, 'r^', markersize=10, label='Goal')
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_aspect('equal')
    ax_xy.grid(True)
    ax_xy.set_title('Robot Trajectory')
    ax_xy.legend()

    # パネル2: 線速度の時系列
    ax_lin = axes[1]
    ax_lin.plot(cmd_vel_timestamps, cmd_vel_linear_xs, 'b-')
    ax_lin.set_xlabel('Time [s]')
    ax_lin.set_ylabel('Linear Velocity [m/s]')
    ax_lin.grid(True)
    ax_lin.set_title('Linear Velocity (cmd_vel)')

    # パネル3: 角速度の時系列
    ax_ang = axes[2]
    ax_ang.plot(cmd_vel_timestamps, cmd_vel_angular_zs, 'b-')
    ax_ang.set_xlabel('Time [s]')
    ax_ang.set_ylabel('Angular Velocity [rad/s]')
    ax_ang.grid(True)
    ax_ang.set_title('Angular Velocity (cmd_vel)')

    plt.tight_layout()
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

    odom_data, cmd_vel_data = read_rosbag(str(bag_path))

    fig = create_figure(odom_data, cmd_vel_data)

    output_path = output_dir / 'navigation_result.png'
    fig.savefig(str(output_path), dpi=150)
    plt.close(fig)

    print(output_path)


if __name__ == '__main__':
    main()
