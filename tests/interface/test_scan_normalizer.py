"""scan_normalizer 单元测试 —— 验证 valid_range 静态方法和 on_scan 规范化逻辑。"""

import math
import sys
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()
_mock_rclpy.ok = MagicMock(return_value=True)
_mock_rclpy.spin = MagicMock()

_mock_sensor_msgs = MagicMock(name="sensor_msgs")

sys.modules["rclpy"] = _mock_rclpy
sys.modules["sensor_msgs"] = _mock_sensor_msgs
sys.modules["sensor_msgs.msg"] = MagicMock()


class FakeTime:
    """模拟 rclpy Time 对象，支持减法运算。"""
    def __init__(self, ns=0):
        self.nanoseconds = ns
    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)
    def now(self):
        return FakeTime(0)


class MockNode:
    """替代 rclpy.node.Node 的 mock 基类。"""

    _param_overrides: dict = {}

    def __init__(self, node_name: str = "", **kwargs):
        self._node_name = node_name
        self._declared_params = {}

    def declare_parameter(self, name, value=None):
        self._declared_params[name] = value

    def get_parameter(self, name):
        mock = MagicMock()
        if name in MockNode._param_overrides:
            mock.value = MockNode._param_overrides[name]
        elif name in self._declared_params:
            mock.value = self._declared_params[name]
        else:
            mock.value = name
        return mock

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        return FakeTime()

    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        self._scan_callback = callback
        return MagicMock()

    def destroy_node(self):
        pass

    @classmethod
    def set_params(cls, **overrides):
        cls._param_overrides = dict(overrides)


_mock_rclpy.node.Node = MockNode
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.node"].Node = MockNode
sys.modules["rclpy.qos"] = MagicMock()

# 添加 scripts 目录到 path（scan_normalizer 是独立脚本，非 Python 包）
import os as _os

_scripts_dir = _os.path.join(_os.path.dirname(__file__), "..", "..", "src", "robot_sensing", "scripts")
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from scan_normalizer import ScanNormalizer


def make_normalizer(**param_overrides):
    """创建 ScanNormalizer 实例的工厂函数。"""
    MockNode.set_params(**param_overrides)
    node = ScanNormalizer()
    node.pub = MagicMock()
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    return node


def make_laserscan(ranges, angle_min=-math.pi, angle_max=math.pi,
                   range_min=0.1, range_max=8.0, intensities=None):
    """构造一个模拟 LaserScan 消息。"""
    msg = MagicMock()
    msg.header = MagicMock()
    msg.angle_min = float(angle_min)
    msg.angle_max = float(angle_max)
    msg.angle_increment = (angle_max - angle_min) / max(len(ranges) - 1, 1)
    msg.time_increment = 0.0
    msg.scan_time = 0.1
    msg.range_min = float(range_min)
    msg.range_max = float(range_max)
    msg.ranges = [float(r) for r in ranges]
    msg.intensities = [float(i) for i in (intensities or [0.0] * len(ranges))]
    return msg


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------


class TestValidRange:
    """验证 valid_range 静态方法。"""

    def test_finite_value_within_range_is_valid(self):
        assert ScanNormalizer.valid_range(1.0, 0.1, 8.0) is True

    def test_inf_is_invalid(self):
        assert ScanNormalizer.valid_range(math.inf, 0.1, 8.0) is False
        assert ScanNormalizer.valid_range(-math.inf, 0.1, 8.0) is False

    def test_nan_is_invalid(self):
        assert ScanNormalizer.valid_range(math.nan, 0.1, 8.0) is False

    def test_below_range_min_is_invalid(self):
        assert ScanNormalizer.valid_range(0.05, 0.1, 8.0) is False

    def test_above_range_max_is_invalid(self):
        assert ScanNormalizer.valid_range(10.0, 0.1, 8.0) is False

    def test_at_range_boundary_is_valid(self):
        assert ScanNormalizer.valid_range(0.1, 0.1, 8.0) is True
        assert ScanNormalizer.valid_range(8.0, 0.1, 8.0) is True


class TestScanNormalizer:
    """验证 on_scan 方法 —— 扫描规范化逻辑。"""

    def test_normalized_output_has_configured_size(self):
        node = make_normalizer(output_size=100)
        scan = make_laserscan([1.0] * 360)

        node.on_scan(scan)

        node.pub.publish.assert_called_once()
        out = node.pub.publish.call_args[0][0]
        assert len(out.ranges) == 100

    def test_invalid_ranges_are_filtered_out(self):
        node = make_normalizer(output_size=360)
        # 一半有效、一半无效（inf）
        ranges = [1.0] * 180 + [math.inf] * 180
        scan = make_laserscan(ranges, range_min=0.1, range_max=8.0)

        node.on_scan(scan)

        out = node.pub.publish.call_args[0][0]
        # 前半部分应为 1.0，后半部分保持 inf（未被写入）
        assert out.ranges[0] == 1.0
        assert out.ranges[179] == 1.0
        assert out.ranges[180] == math.inf

    def test_closest_range_wins_for_overlapping_bins(self):
        node = make_normalizer(output_size=10, angle_min=-1.0, angle_max=1.0)
        # 两个点映射到同一个 bin — 取最近（较小）的
        scan = make_laserscan([0.5, 2.0], angle_min=-1.0, angle_max=1.0, range_min=0.1, range_max=8.0)

        node.on_scan(scan)

        out = node.pub.publish.call_args[0][0]
        # 两个点应映射到不同 bin（各有 2/(10-1)=0.222... 的增量）
        assert out.ranges[0] == 0.5
        assert out.ranges[9] == 2.0

    def test_output_preserves_header(self):
        node = make_normalizer(output_size=360)
        scan = make_laserscan([1.0] * 360)
        scan.header.stamp = MagicMock()
        scan.header.frame_id = "laser"

        node.on_scan(scan)

        out = node.pub.publish.call_args[0][0]
        assert out.header is scan.header

    def test_out_of_bounds_angle_is_skipped(self):
        node = make_normalizer(output_size=10, angle_min=-0.5, angle_max=0.5)
        # angle_max - angle_min = 1.0, increment = 1.0/9 ≈ 0.111
        # 所有点角度在 [-1, 1] 范围内，大部分会越界
        scan = make_laserscan([1.0] * 360, angle_min=-1.0, angle_max=1.0, range_min=0.1, range_max=8.0)

        node.on_scan(scan)

        out = node.pub.publish.call_args[0][0]
        # 只有落在 [-0.5, 0.5] 内的点会被保留
        valid_count = sum(1 for r in out.ranges if r != math.inf)
        assert valid_count > 0
        assert valid_count < 360


@pytest.mark.parametrize(
    "scan_time,time_increment",
    [
        (0.0, 0.0),
        (-0.1, 0.0),
        (math.nan, 0.0),
        (math.inf, 0.0),
        (0.1, -0.001),
        (0.1, math.nan),
        (0.1, math.inf),
        (0.1, 0.2),
    ],
)
def test_invalid_timing_is_not_forwarded(scan_time, time_increment):
    node = make_normalizer()
    scan = make_laserscan([1.0] * 425)
    scan.scan_time = scan_time
    scan.time_increment = time_increment

    node.on_scan(scan)

    node.pub.publish.assert_not_called()
    assert node.message_count == 0


def test_valid_scan_recovers_after_zero_period_startup_frame():
    node = make_normalizer()
    startup = make_laserscan([1.0] * 425)
    startup.scan_time = 0.0
    node.on_scan(startup)
    valid = make_laserscan([1.0] * 425)
    node.on_scan(valid)

    node.pub.publish.assert_called_once()
    assert node.pub.publish.call_args[0][0].header is valid.header
    assert node.message_count == 1
