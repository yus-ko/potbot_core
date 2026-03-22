"""
analyze_rosbag.py の単体テスト

テスト対象:
  - read_rosbag()   : rosbag2 からトピックデータを読み込めるか
  - create_figure() : 読み込んだデータから matplotlib Figure が生成されるか
  - main() (統合)   : navigation_result.png が出力されるか

rosbags ライブラリで最小限の合成 rosbag2 を生成してテストデータとして使用する。
Gazebo / Nav2 などの実行環境は不要。
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pytest

# テスト対象モジュールを import できるようパスを追加
sys.path.insert(0, str(Path(__file__).parent))
import analyze_rosbag


# ---------------------------------------------------------------------------
# rosbags TypeStore (モジュールレベルで一度だけ初期化)
# ---------------------------------------------------------------------------

from rosbags.typesys import Stores, get_typestore as _get_typestore
_TS = _get_typestore(Stores.ROS2_HUMBLE)


def _make_odometry(x: float, y: float) -> bytes:
    """nav_msgs/msg/Odometry を CDR シリアライズして返す。"""
    H      = _TS.types['std_msgs/msg/Header']
    Stamp  = _TS.types['builtin_interfaces/msg/Time']
    Point  = _TS.types['geometry_msgs/msg/Point']
    Q      = _TS.types['geometry_msgs/msg/Quaternion']
    Pose   = _TS.types['geometry_msgs/msg/Pose']
    PWC    = _TS.types['geometry_msgs/msg/PoseWithCovariance']
    TWC    = _TS.types['geometry_msgs/msg/TwistWithCovariance']
    Twist  = _TS.types['geometry_msgs/msg/Twist']
    Vec3   = _TS.types['geometry_msgs/msg/Vector3']
    Odom   = _TS.types['nav_msgs/msg/Odometry']

    msg = Odom(
        header=H(stamp=Stamp(sec=0, nanosec=0), frame_id='odom'),
        child_frame_id='base_footprint',
        pose=PWC(
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36),
        ),
        twist=TWC(
            twist=Twist(
                linear=Vec3(x=0.0, y=0.0, z=0.0),
                angular=Vec3(x=0.0, y=0.0, z=0.0),
            ),
            covariance=np.zeros(36),
        ),
    )
    return _TS.serialize_cdr(msg, 'nav_msgs/msg/Odometry')


def _make_twist(linear_x: float, angular_z: float) -> bytes:
    """geometry_msgs/msg/Twist を CDR シリアライズして返す。"""
    Vec3  = _TS.types['geometry_msgs/msg/Vector3']
    Twist = _TS.types['geometry_msgs/msg/Twist']

    msg = Twist(
        linear=Vec3(x=linear_x, y=0.0, z=0.0),
        angular=Vec3(x=0.0, y=0.0, z=angular_z),
    )
    return _TS.serialize_cdr(msg, 'geometry_msgs/msg/Twist')


def _make_path(points: list) -> bytes:
    """nav_msgs/msg/Path を CDR シリアライズして返す。"""
    H    = _TS.types['std_msgs/msg/Header']
    Stamp = _TS.types['builtin_interfaces/msg/Time']
    Point = _TS.types['geometry_msgs/msg/Point']
    Q    = _TS.types['geometry_msgs/msg/Quaternion']
    Pose = _TS.types['geometry_msgs/msg/Pose']
    PS   = _TS.types['geometry_msgs/msg/PoseStamped']
    Path = _TS.types['nav_msgs/msg/Path']

    poses = [
        PS(
            header=H(stamp=Stamp(sec=0, nanosec=0), frame_id='map'),
            pose=Pose(
                position=Point(x=px, y=py, z=0.0),
                orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        for px, py in points
    ]
    msg = Path(
        header=H(stamp=Stamp(sec=0, nanosec=0), frame_id='map'),
        poses=poses,
    )
    return _TS.serialize_cdr(msg, 'nav_msgs/msg/Path')


# ---------------------------------------------------------------------------
# テスト用合成 rosbag2 フィクスチャ
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def synthetic_bag(tmp_path_factory):
    """最小限の合成 rosbag2 を生成し、そのパスを返す。

    /odom, /cmd_vel, /plan の3トピックを含む。
    各トピックに5メッセージずつ書き込む。
    """
    from rosbags.rosbag2 import Writer

    # Writer は既存ディレクトリを拒否するため mkdir せずに渡す
    bag_dir = tmp_path_factory.mktemp('rosbag2') / 'test_bag'

    with Writer(str(bag_dir), version=9) as writer:
        conn_odom = writer.add_connection(
            '/odom',
            'nav_msgs/msg/Odometry',
            typestore=_TS,
        )
        conn_cmd = writer.add_connection(
            '/cmd_vel',
            'geometry_msgs/msg/Twist',
            typestore=_TS,
        )
        conn_plan = writer.add_connection(
            '/plan',
            'nav_msgs/msg/Path',
            typestore=_TS,
        )

        base_ns = 1_000_000_000  # 1秒 = 1e9 ns

        for i in range(5):
            t_ns = base_ns + i * 500_000_000  # 0.5秒ごと
            angle = i * 0.2

            writer.write(
                conn_odom,
                t_ns,
                _make_odometry(
                    x=math.cos(angle) * i * 0.3,
                    y=math.sin(angle) * i * 0.3,
                ),
            )
            writer.write(
                conn_cmd,
                t_ns + 1_000,
                _make_twist(linear_x=0.1 * (i + 1), angular_z=0.05 * i),
            )
            plan_points = [(j * 0.1 + i * 0.2, j * 0.05) for j in range(5)]
            writer.write(
                conn_plan,
                t_ns + 2_000,
                _make_path(plan_points),
            )

    return bag_dir


# ---------------------------------------------------------------------------
# テストケース
# ---------------------------------------------------------------------------

class TestReadRosbag:
    """read_rosbag() の動作検証"""

    def test_returns_odom_data(self, synthetic_bag):
        """odom データが読み込まれること"""
        odom, cmd, plan, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        timestamps, xs, ys = odom
        assert len(timestamps) == 5, f"odometry メッセージ数が不正: {len(timestamps)}"
        assert len(xs) == len(ys) == len(timestamps)

    def test_returns_cmd_vel_data(self, synthetic_bag):
        """cmd_vel データが読み込まれること"""
        odom, cmd, plan, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        timestamps, linear_xs, angular_zs = cmd
        assert len(timestamps) == 5, f"cmd_vel メッセージ数が不正: {len(timestamps)}"
        assert len(linear_xs) == len(angular_zs) == len(timestamps)

    def test_returns_plan_data(self, synthetic_bag):
        """/plan データが読み込まれること"""
        odom, cmd, plan, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        plan_timestamps, plan_paths = plan
        assert len(plan_timestamps) == 5, f"/plan メッセージ数が不正: {len(plan_timestamps)}"
        assert all(len(path) > 0 for path in plan_paths), "経路データが空"

    def test_timestamps_are_relative(self, synthetic_bag):
        """タイムスタンプが0始まりの相対時刻であること"""
        odom, cmd, plan, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        timestamps, _, _ = odom
        assert timestamps[0] == pytest.approx(0.0, abs=1e-6), \
            f"最初のタイムスタンプが0ではない: {timestamps[0]}"
        assert timestamps[-1] > 0.0

    def test_bag_start_ns_is_positive(self, synthetic_bag):
        """bag_start_ns が正の値であること"""
        _, _, _, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        assert start_ns > 0


class TestCreateFigure:
    """create_figure() の動作検証"""

    def test_returns_figure_object(self, synthetic_bag):
        """matplotlib Figure が返されること"""
        odom, cmd, plan, _ = analyze_rosbag.read_rosbag(str(synthetic_bag))
        fig = analyze_rosbag.create_figure(odom, cmd, plan)
        assert isinstance(fig, plt.Figure)
        plt.close(fig)

    def test_figure_has_4_panels(self, synthetic_bag):
        """リソースなし時に4パネルの図が生成されること。
        /plan データがある場合はカラーバーが1軸追加されるため axes >= 4 で検証する。
        """
        odom, cmd, plan, _ = analyze_rosbag.read_rosbag(str(synthetic_bag))
        fig = analyze_rosbag.create_figure(odom, cmd, plan)
        assert len(fig.axes) >= 4, f"パネル数が不正: {len(fig.axes)}"
        plt.close(fig)

    def test_figure_has_6_panels_with_resources(self, synthetic_bag):
        """resources を渡すと少なくとも6パネルの図が生成されること。
        カラーバー軸が追加される場合があるため axes >= 6 で検証する。
        """
        odom, cmd, plan, start_ns = analyze_rosbag.read_rosbag(str(synthetic_bag))
        resources = {
            'time_sec': [0.0, 1.0, 2.0],
            'system_cpu_percent': [10.0, 20.0, 15.0],
            'system_memory_used_mb': [200.0, 210.0, 205.0],
        }
        fig = analyze_rosbag.create_figure(odom, cmd, plan, resources=resources)
        assert len(fig.axes) >= 6, f"パネル数が不正: {len(fig.axes)}"
        plt.close(fig)


class TestPngOutput:
    """analyze_rosbag.py のエンドツーエンド出力検証"""

    def test_png_is_created(self, synthetic_bag, tmp_path):
        """navigation_result.png が生成されること"""
        out_dir = tmp_path / 'results'
        out_dir.mkdir()

        odom, cmd, plan, _ = analyze_rosbag.read_rosbag(str(synthetic_bag))
        fig = analyze_rosbag.create_figure(odom, cmd, plan)
        output_path = out_dir / 'navigation_result.png'
        fig.savefig(str(output_path), dpi=72)
        plt.close(fig)

        assert output_path.exists(), "navigation_result.png が生成されていない"
        assert output_path.stat().st_size > 0, "navigation_result.png が空ファイル"

    def test_png_via_main(self, synthetic_bag, tmp_path, monkeypatch):
        """main() 経由で navigation_result.png が生成されること"""
        out_dir = tmp_path / 'results'
        out_dir.mkdir()

        monkeypatch.setattr(
            'sys.argv',
            [
                'analyze_rosbag.py',
                '--bag-path', str(synthetic_bag),
                '--output-dir', str(out_dir),
                '--goal-x', '2.0',
                '--goal-y', '0.5',
            ],
        )

        analyze_rosbag.main()

        output_path = out_dir / 'navigation_result.png'
        assert output_path.exists(), "main() 後に navigation_result.png が生成されていない"
        assert output_path.stat().st_size > 0, "navigation_result.png が空ファイル"
