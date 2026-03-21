#!/usr/bin/env python3
"""ナビゲーション実行中のシステムリソースをCSVに記録するサイドカースクリプト。

rosbag record と並行して起動し、CPU・メモリ使用量を時系列でCSVに保存する。
analyze_rosbag.py の --resources-csv オプションと組み合わせて使用する。

使用例:
    # ターミナル1: rosbag記録開始
    ros2 bag record -o nav_test /odom /cmd_vel

    # ターミナル2: リソース記録を同時に開始 (同じタイミングで起動する)
    uv run python resource_monitor.py --output nav_test_resources.csv

    # 両方をCtrl+Cで停止後、解析
    uv run python analyze_rosbag.py --bag-path nav_test --resources-csv nav_test_resources.csv
"""

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import psutil


# 監視対象プロセス名のデフォルト (部分一致)
DEFAULT_PROCESSES = [
    'nav2_planner',
    'nav2_controller',
    'planner_server',
    'controller_server',
]

_running = True


def _handle_signal(signum, frame):
    global _running
    _running = False


def parse_args():
    parser = argparse.ArgumentParser(
        description='ナビゲーション実行中のリソース使用量をCSVに記録する'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='resources.csv',
        help='出力CSVファイルパス (デフォルト: resources.csv)',
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=0.1,
        help='サンプリング間隔 [秒] (デフォルト: 0.1 = 10Hz)',
    )
    parser.add_argument(
        '--processes',
        type=str,
        nargs='*',
        default=DEFAULT_PROCESSES,
        help=f'監視するプロセス名 (部分一致, デフォルト: {DEFAULT_PROCESSES})',
    )
    return parser.parse_args()


def find_target_processes(target_names):
    """名前が部分一致するプロセスを検索して返す。

    Args:
        target_names: 監視対象プロセス名のリスト (部分一致)。

    Returns:
        dict: {表示名: psutil.Process} のマッピング。
    """
    found = {}
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            proc_name = proc.info['name'] or ''
            cmdline = ' '.join(proc.info['cmdline'] or [])
            for target in target_names:
                if target in proc_name or target in cmdline:
                    if target not in found:
                        found[target] = proc
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return found


def build_csv_header(process_names):
    """CSVヘッダー行を生成する。"""
    header = ['time_ns', 'system_cpu_percent', 'system_memory_used_mb']
    for name in process_names:
        header.append(f'{name}_cpu_percent')
        header.append(f'{name}_memory_mb')
    return header


def sample_resources(target_procs):
    """現時点のリソース使用量を計測してdictで返す。

    Args:
        target_procs: {表示名: psutil.Process} のマッピング。

    Returns:
        dict: CSVヘッダーに対応する計測値。
    """
    row = {
        'time_ns': time.time_ns(),
        'system_cpu_percent': psutil.cpu_percent(interval=None),
        'system_memory_used_mb': psutil.virtual_memory().used / 1024 / 1024,
    }

    dead_keys = []
    for name, proc in target_procs.items():
        try:
            cpu = proc.cpu_percent(interval=None)
            mem_mb = proc.memory_info().rss / 1024 / 1024
            row[f'{name}_cpu_percent'] = cpu
            row[f'{name}_memory_mb'] = mem_mb
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            row[f'{name}_cpu_percent'] = float('nan')
            row[f'{name}_memory_mb'] = float('nan')
            dead_keys.append(name)

    for key in dead_keys:
        del target_procs[key]

    return row


def main():
    global _running

    args = parse_args()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # 対象プロセスを検索
    target_procs = find_target_processes(args.processes)
    if target_procs:
        print(f'監視対象プロセス: {list(target_procs.keys())}')
    else:
        print('警告: 監視対象プロセスが見つかりません。システム全体のみ記録します。')
        print(f'  検索対象: {args.processes}')

    # cpu_percent の初回呼び出しは常に 0.0 を返すため、捨てる
    psutil.cpu_percent(interval=None)
    for proc in target_procs.values():
        try:
            proc.cpu_percent(interval=None)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    time.sleep(args.interval)

    header = build_csv_header(list(args.processes))
    rows = []

    print(f'記録開始 -> {output_path}  (Ctrl+C で停止)')

    while _running:
        loop_start = time.monotonic()

        row = sample_resources(target_procs)

        # プロセスが途中から起動した場合に再検索
        missing = [n for n in args.processes if n not in target_procs]
        if missing:
            newly_found = find_target_processes(missing)
            if newly_found:
                print(f'新たに検出: {list(newly_found.keys())}')
                target_procs.update(newly_found)

        rows.append(row)

        elapsed = time.monotonic() - loop_start
        sleep_time = args.interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    # CSV書き出し
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)

    print(f'記録完了: {len(rows)} サンプル -> {output_path}')


if __name__ == '__main__':
    main()
