#!/usr/bin/env python3
"""rosbag2 データを解析し、ロボットの軌跡・速度指令・計画経路を可視化するスクリプト。

rosbags ライブラリを使用して rosbag2 (sqlite3形式) を読み込み、
/odom、/cmd_vel、/plan トピックからデータを抽出して matplotlib で4パネルの
図を生成・保存する。

--resources-csv を指定すると resource_monitor.py が出力したCSVも読み込み、
CPU・メモリ使用量のパネルを追加した6パネル図を生成する。
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
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
        '--resources-csv',
        type=str,
        default=None,
        help='resource_monitor.py が出力したCSVファイルパス (指定時はCPU/メモリパネルを追加)',
    )
    parser.add_argument(
        '--no-auto-zoom',
        action='store_true',
        default=False,
        help='軌跡パネルの表示範囲自動調整を無効化する (デフォルト: 自動調整ON)',
    )
    return parser.parse_args()


def read_resources_csv(csv_path, bag_start_ns):
    """resource_monitor.py が出力したCSVを読み込み、bagの時間軸に揃える。

    タイムスタンプはどちらも wall-clock (UNIX epoch nanoseconds) のため、
    差分で相対時刻に変換する。

    Args:
        csv_path: CSVファイルのパス。
        bag_start_ns: bagの最初のメッセージのタイムスタンプ [ns]。

    Returns:
        dict: {列名: list} の形式。時刻列は 'time_sec' (float, bag開始からの秒数)。
              bagより前のサンプルは time_sec < 0 になるため呼び出し側でフィルタ可能。
    """
    result = {}
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        return result

    for key in rows[0].keys():
        result[key] = []

    for row in rows:
        for key, val in row.items():
            try:
                result[key].append(float(val))
            except (ValueError, TypeError):
                result[key].append(float('nan'))

    # time_ns → bagの開始を0秒とした相対時刻に変換
    result['time_sec'] = [
        (ns - bag_start_ns) / 1e9
        for ns in result.get('time_ns', [])
    ]
    return result


def read_rosbag(bag_path):
    """rosbag2 からトピックデータを読み込む。

    Args:
        bag_path: rosbag2 ディレクトリのパス。

    Returns:
        odom_data: (timestamps, xs, ys, linear_xs, angular_zs) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。
        plan_data: (plan_timestamps, plan_paths) のタプル。
            plan_timestamps: 各 /plan メッセージのタイムスタンプリスト [s]。
            plan_paths: 各 /plan メッセージの [(x, y), ...] リスト。
        goal_pose: (goal_x, goal_y) のタプル。/test/goal_pose が未記録の場合は None。
        map_data: OccupancyGrid の描画用データ dict。/map が未記録の場合は None。
            'image': RGBA numpy 配列 (height x width x 4)。
            'extent': [x_min, x_max, y_min, y_max] (world 座標 [m])。
        bag_start_ns: bagの最初のメッセージのタイムスタンプ [ns] (wall-clock)。
    """
    odom_timestamps = []
    odom_xs = []
    odom_ys = []
    odom_linear_xs = []
    odom_angular_zs = []

    cmd_vel_timestamps = []
    cmd_vel_linear_xs = []
    cmd_vel_angular_zs = []

    plan_timestamps = []
    plan_paths = []

    goal_pose = None
    map_data = None

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
                odom_linear_xs.append(msg.twist.twist.linear.x)
                odom_angular_zs.append(msg.twist.twist.angular.z)

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

            elif connection.topic == '/test/goal_pose' and goal_pose is None:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                goal_pose = (msg.pose.position.x, msg.pose.position.y)

            elif connection.topic == '/map':
                # 最後のマップメッセージを使用（AMCLが収束後のものが最も正確）
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                info = msg.info
                width = info.width
                height = info.height
                resolution = info.resolution
                origin_x = info.origin.position.x
                origin_y = info.origin.position.y

                # OccupancyGrid データを2次元配列に変換
                grid = np.array(msg.data, dtype=np.int8).reshape(height, width)

                # RGBA画像に変換: 不明(-1)=灰色, 自由(0)=白, 占有(100)=黒
                rgba = np.zeros((height, width, 4), dtype=np.float32)
                free_mask = grid == 0
                occ_mask = grid == 100
                unk_mask = grid == -1

                rgba[free_mask] = [1.0, 1.0, 1.0, 1.0]   # 白: 自由空間
                rgba[occ_mask] = [0.0, 0.0, 0.0, 1.0]    # 黒: 障害物
                rgba[unk_mask] = [0.5, 0.5, 0.5, 0.6]    # 灰色: 未知領域

                x_min = origin_x
                x_max = origin_x + width * resolution
                y_min = origin_y
                y_max = origin_y + height * resolution
                map_data = {
                    'image': rgba,
                    'extent': [x_min, x_max, y_min, y_max],
                }

    odom_data = (odom_timestamps, odom_xs, odom_ys, odom_linear_xs, odom_angular_zs)
    cmd_vel_data = (cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs)
    plan_data = (plan_timestamps, plan_paths)
    bag_start_ns = start_time if start_time is not None else 0
    return odom_data, cmd_vel_data, plan_data, goal_pose, map_data, bag_start_ns


def _plot_resource_panels(axes, resources, panel_offset):
    """リソースデータをCPU・メモリの2パネルに描画する。

    Args:
        axes: matplotlib の Axes 配列。
        resources: read_resources_csv の戻り値 dict。
        panel_offset: CPU パネルのインデックス (memory は +1)。
    """
    t = resources.get('time_sec', [])
    if not t:
        return

    # CPU パネル
    ax_cpu = axes[panel_offset]
    sys_cpu = resources.get('system_cpu_percent', [])
    if sys_cpu:
        ax_cpu.plot(t, sys_cpu, color='steelblue', linewidth=1.2, label='System CPU')

    # プロセス個別CPU (system_* 以外の *_cpu_percent 列)
    proc_cpu_keys = [
        k for k in resources
        if k.endswith('_cpu_percent') and k != 'system_cpu_percent'
    ]
    colors = plt.cm.tab10.colors
    for i, key in enumerate(proc_cpu_keys):
        vals = resources[key]
        label = key.replace('_cpu_percent', '')
        ax_cpu.plot(t, vals, color=colors[(i + 1) % 10],
                    linewidth=1.0, linestyle='--', label=label)

    ax_cpu.set_xlabel('Time [s]')
    ax_cpu.set_ylabel('CPU [%]')
    ax_cpu.set_ylim(bottom=0)
    ax_cpu.grid(True)
    ax_cpu.set_title('CPU Usage')
    ax_cpu.legend(fontsize='small')

    # メモリパネル
    ax_mem = axes[panel_offset + 1]
    sys_mem = resources.get('system_memory_used_mb', [])
    if sys_mem:
        ax_mem.plot(t, sys_mem, color='darkorange', linewidth=1.2, label='System Memory')

    proc_mem_keys = [
        k for k in resources
        if k.endswith('_memory_mb')
    ]
    for i, key in enumerate(proc_mem_keys):
        vals = resources[key]
        label = key.replace('_memory_mb', '')
        ax_mem.plot(t, vals, color=colors[(i + 1) % 10],
                    linewidth=1.0, linestyle='--', label=label)

    ax_mem.set_xlabel('Time [s]')
    ax_mem.set_ylabel('Memory [MB]')
    ax_mem.set_ylim(bottom=0)
    ax_mem.grid(True)
    ax_mem.set_title('Memory Usage')
    ax_mem.legend(fontsize='small')


def create_figure(odom_data, cmd_vel_data, plan_data, goal_x=None, goal_y=None,
                  map_data=None, resources=None, auto_zoom=True):
    """4〜6パネルの図を生成する。

    Args:
        odom_data: (timestamps, xs, ys, linear_xs, angular_zs) のタプル。
        cmd_vel_data: (timestamps, linear_xs, angular_zs) のタプル。
        plan_data: (plan_timestamps, plan_paths) のタプル。
        goal_x: ゴール地点のX座標 [m]。None の場合はゴールマーカーを描画しない。
        goal_y: ゴール地点のY座標 [m]。None の場合はゴールマーカーを描画しない。
        map_data: read_rosbag の戻り値 map_data dict。None の場合はマップ背景なし。
        resources: read_resources_csv の戻り値 dict。None の場合はリソースパネルなし。
        auto_zoom: True の場合、軌跡パネルの表示範囲をロボット軌跡に合わせて正方形に自動調整する。

    Returns:
        matplotlib の Figure オブジェクト。
    """
    odom_timestamps, odom_xs, odom_ys, odom_linear_xs, odom_angular_zs = odom_data
    cmd_vel_timestamps, cmd_vel_linear_xs, cmd_vel_angular_zs = cmd_vel_data
    plan_timestamps, plan_paths = plan_data

    n_panels = 6 if resources else 4
    fig_height = 24 if resources else 16
    fig, axes = plt.subplots(n_panels, 1, figsize=(10, fig_height), constrained_layout=True)

    ax_xy = axes[0]

    # マップを背景として描画（imshow は Y軸が上下逆なので origin='lower' を指定）
    if map_data is not None:
        ax_xy.imshow(
            map_data['image'],
            extent=map_data['extent'],
            origin='lower',
            aspect='equal',
            zorder=0,
        )

    for path in plan_paths[:-1]:
        if path:
            xs, ys = zip(*path)
            ax_xy.plot(xs, ys, color='gray', linewidth=0.8, alpha=0.5, zorder=2)

    if plan_paths:
        latest_path = plan_paths[-1]
        if latest_path:
            xs, ys = zip(*latest_path)
            ax_xy.plot(xs, ys, 'r-', linewidth=1.5, label='Plan (latest)', zorder=3)

    if len(plan_paths) > 1:
        ax_xy.plot([], [], color='gray', linewidth=0.8, alpha=0.5, label='Plan (old)')

    ax_xy.plot(odom_xs, odom_ys, 'b-', label='Trajectory (odom)', zorder=4)
    if odom_xs:
        ax_xy.plot(odom_xs[0], odom_ys[0], 'go', markersize=10, label='Start', zorder=5)

    if goal_x is not None and goal_y is not None:
        ax_xy.plot(goal_x, goal_y, 'r^', markersize=10, label='Goal', zorder=5)
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_aspect('equal')
    ax_xy.grid(True, alpha=0.4, zorder=1)
    ax_xy.set_title('Robot Trajectory' + (' (with map)' if map_data else ''))
    ax_xy.legend()

    if auto_zoom and odom_xs and odom_ys:
        # 軌跡・ゴール・開始点を包含する正方形領域に表示範囲を設定
        ref_xs = list(odom_xs)
        ref_ys = list(odom_ys)
        if goal_x is not None:
            ref_xs.append(goal_x)
        if goal_y is not None:
            ref_ys.append(goal_y)
        x_min, x_max = min(ref_xs), max(ref_xs)
        y_min, y_max = min(ref_ys), max(ref_ys)
        span = max(x_max - x_min, y_max - y_min)
        margin = span * 0.1 + 0.5  # 10%余白 + 最低0.5m
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        half = span / 2 + margin
        ax_xy.set_xlim(cx - half, cx + half)
        ax_xy.set_ylim(cy - half, cy + half)

    ax_lin = axes[1]
    ax_lin.plot(cmd_vel_timestamps, cmd_vel_linear_xs, 'b-', label='cmd_vel')
    if odom_linear_xs:
        ax_lin.plot(odom_timestamps, odom_linear_xs, 'r-', linewidth=0.8,
                    alpha=0.7, label='odom')
    ax_lin.set_xlabel('Time [s]')
    ax_lin.set_ylabel('Linear Velocity [m/s]')
    ax_lin.grid(True)
    ax_lin.set_title('Linear Velocity')
    ax_lin.legend(fontsize='small')

    ax_ang = axes[2]
    ax_ang.plot(cmd_vel_timestamps, cmd_vel_angular_zs, 'b-', label='cmd_vel')
    if odom_angular_zs:
        ax_ang.plot(odom_timestamps, odom_angular_zs, 'r-', linewidth=0.8,
                    alpha=0.7, label='odom')
    ax_ang.set_xlabel('Time [s]')
    ax_ang.set_ylabel('Angular Velocity [rad/s]')
    ax_ang.grid(True)
    ax_ang.set_title('Angular Velocity')
    ax_ang.legend(fontsize='small')

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

    if resources:
        _plot_resource_panels(axes, resources, panel_offset=4)

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

    odom_data, cmd_vel_data, plan_data, goal_pose, map_data, bag_start_ns = read_rosbag(str(bag_path))

    if goal_pose is not None:
        goal_x, goal_y = goal_pose
        print(f'ゴール位置をrosbagから取得しました: x={goal_x}, y={goal_y}')
    else:
        print('警告: /test/goal_pose がrosbagに含まれていません。ゴールマーカーを描画しません。',
              file=sys.stderr)
        goal_x, goal_y = None, None

    if map_data is not None:
        print('マップをrosbagから取得しました。軌跡パネルの背景に描画します。')
    else:
        print('警告: /map がrosbagに含まれていません。マップ背景なしで描画します。',
              file=sys.stderr)

    resources = None
    if args.resources_csv:
        csv_path = Path(args.resources_csv)
        if not csv_path.exists():
            print(f'警告: リソースCSVが見つかりません: {csv_path}', file=sys.stderr)
        else:
            resources = read_resources_csv(str(csv_path), bag_start_ns)

    fig = create_figure(odom_data, cmd_vel_data, plan_data,
                        goal_x=goal_x, goal_y=goal_y,
                        map_data=map_data,
                        resources=resources,
                        auto_zoom=not args.no_auto_zoom)

    output_path = output_dir / 'navigation_result.png'
    fig.savefig(str(output_path), dpi=150)
    plt.close(fig)

    print(output_path)


if __name__ == '__main__':
    main()
