from types import SimpleNamespace

import pytest

from lerobot.utils.constants import OBS_STR
from lerobot.utils.feature_utils import hw_to_dataset_features


@pytest.mark.parametrize("module", ["lerobot_find_joint_limits", "lerobot_setup_motors"])
def test_unsupported_hardware_tools_do_not_advertise_piper(module):
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, "-m", f"lerobot.scripts.{module}", "--help"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "piper_follower" not in result.stdout
    assert "piper_leader" not in result.stdout


def test_native_factories_preserve_piper_configuration(monkeypatch, tmp_path):
    import draccus

    from lerobot.robots import make_robot_from_config
    from lerobot.robots.piper_follower import piper_follower
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig
    from lerobot.teleoperators import make_teleoperator_from_config
    from lerobot.teleoperators.piper_leader import piper_leader

    config = draccus.parse(
        TeleoperateConfig,
        args=[
            "--robot.type=piper_follower",
            "--teleop.type=piper_leader",
        ],
    )
    follower_config = config.robot
    leader_config = config.teleop
    assert follower_config.motors.can_name == "can0"
    assert leader_config.motors.can_name == "can_master1"
    assert [camera.serial_number_or_name for camera in follower_config.cameras.values()] == [
        "230322272075",
        "233622078698",
    ]
    assert all(
        (camera.width, camera.height, camera.fps) == (640, 480, 30)
        for camera in follower_config.cameras.values()
    )

    # Exercise the real native factory and constructors, with no SDK or camera construction.
    bus = SimpleNamespace(is_connected=False)
    monkeypatch.setattr(piper_follower, "PiperMotorsBus", lambda config: bus)
    monkeypatch.setattr(piper_leader, "PiperMotorsBus", lambda config: bus)
    monkeypatch.setattr(piper_follower, "make_cameras_from_configs", lambda configs: {})
    follower_config.calibration_dir = tmp_path / "follower"
    leader_config.calibration_dir = tmp_path / "leader"
    follower = make_robot_from_config(follower_config)
    leader = make_teleoperator_from_config(leader_config)
    assert isinstance(follower, piper_follower.PiperFollower)
    assert isinstance(leader, piper_leader.PiperLeader)
    assert follower.bus is leader.bus is bus
    assert follower.action_features == leader.action_features


def test_custom_piper_settings_are_not_replaced():
    import draccus

    from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
    from lerobot.robots.piper_follower import PiperFollowerConfig
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

    config = draccus.parse(
        TeleoperateConfig,
        args=[
            "--robot.type=piper_follower",
            "--teleop.type=piper_leader",
            "--robot.motors.can_name=can_custom",
            "--robot.cameras={front: {type: opencv, index_or_path: 0, width: 320, height: 240, fps: 15}}",
        ],
    ).robot
    assert isinstance(config, PiperFollowerConfig)
    assert config.motors.can_name == "can_custom"
    assert list(config.cameras) == ["front"]
    other = PiperFollowerConfig()
    config.motors.motors.pop("gripper")
    assert "gripper" in other.motors.motors


def test_piper_config_import_without_sdk():
    import subprocess
    import sys

    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import sys
from lerobot.utils import import_utils
import_utils._pyagxarm_available = False
import_utils._require_package_cache['pyAgxArm'] = False
sys.modules['pyAgxArm'] = None
from lerobot.robots.piper_follower import PiperFollowerConfig
from lerobot.teleoperators.piper_leader import PiperLeaderConfig
from lerobot.motors.piper import PiperMotorsBus
assert PiperFollowerConfig().type == 'piper_follower'
assert PiperLeaderConfig().type == 'piper_leader'
try:
    PiperMotorsBus(PiperFollowerConfig().motors)
except ImportError as error:
    assert 'lerobot[piper]' in str(error)
else:
    raise AssertionError('Missing SDK must fail before accessing hardware')
""",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr


def test_piper_registers_follower_and_leader_configs():
    from lerobot.robots.piper_follower import PiperFollowerConfig
    from lerobot.teleoperators.piper_leader import PiperLeaderConfig

    assert PiperFollowerConfig().type == "piper_follower"
    assert PiperLeaderConfig().type == "piper_leader"


def test_piper_camera_features_keep_raw_camera_keys():
    from lerobot.robots.piper_follower import PiperFollower

    robot = object.__new__(PiperFollower)
    robot.cameras = {"pikaGripperDepthCamera": SimpleNamespace(height=480, width=640)}

    features = hw_to_dataset_features(robot._cameras_ft, OBS_STR, use_video=False)

    assert set(features) == {"observation.images.pikaGripperDepthCamera"}


def test_piper_follower_default_cameras_match_policy_feature_keys():
    from lerobot.robots.piper_follower import PiperFollowerConfig

    config = PiperFollowerConfig()

    assert list(config.cameras) == ["pikaGripperDepthCamera", "sideDepthCamera"]


def test_enable_state_returns_when_joint_status_matches_request():
    from lerobot.motors.piper import PiperMotorsBus

    bus = object.__new__(PiperMotorsBus)
    bus.piper = SimpleNamespace(
        enable=lambda: None,
        get_joints_enable_status_list=lambda: [True] * 6,
    )

    assert bus._set_enable_state(True, timeout_s=0.1) is True


def test_follower_disconnect_waits_for_safe_pose_before_disabling(monkeypatch):
    from lerobot.robots.piper_follower import piper_follower

    calls = []
    follower = object.__new__(piper_follower.PiperFollower)
    follower.bus = SimpleNamespace(
        is_connected=True,
        safe_disconnect=lambda: calls.append("safe_disconnect"),
        connect=lambda enable: calls.append(("connect", enable)),
    )
    follower.cameras = {}
    monkeypatch.setattr(piper_follower.time, "sleep", lambda seconds: calls.append(("sleep", seconds)))

    follower.disconnect()

    assert calls == ["safe_disconnect", ("sleep", 5.0), ("connect", False)]


def test_follower_connect_disables_bus_when_enable_fails():
    from lerobot.robots.piper_follower import PiperFollower

    calls = []
    follower = object.__new__(PiperFollower)
    follower.bus = SimpleNamespace(
        is_connected=False,
        connect=lambda enable: calls.append(("connect", enable)) or False,
    )
    follower.cameras = {}

    with pytest.raises(ConnectionError, match="Failed to enable Piper follower"):
        follower.connect()

    assert calls == [("connect", True), ("connect", False)]


def test_leader_connect_disables_bus_when_enable_fails():
    from lerobot.teleoperators.piper_leader import PiperLeader

    calls = []
    leader = object.__new__(PiperLeader)
    leader.bus = SimpleNamespace(
        is_connected=False,
        connect=lambda enable: calls.append(("connect", enable)) or False,
    )

    with pytest.raises(ConnectionError, match="Failed to enable Piper leader"):
        leader.connect()

    assert calls == [("connect", True), ("connect", False)]


def test_leader_connect_disables_bus_when_calibration_fails():
    from lerobot.teleoperators.piper_leader import PiperLeader

    calls = []
    leader = object.__new__(PiperLeader)
    bus = SimpleNamespace(is_connected=False, is_calibrated=False)

    def connect(enable):
        calls.append(("connect", enable))
        bus.is_connected = enable
        return True

    leader.bus = bus
    bus.connect = connect
    leader.calibrate = lambda: (_ for _ in ()).throw(RuntimeError("calibration failed"))

    with pytest.raises(RuntimeError, match="calibration failed"):
        leader.connect()

    assert calls == [("connect", True), ("connect", False)]


def test_follower_disconnect_releases_cameras_when_bus_disable_fails(monkeypatch):
    from lerobot.robots.piper_follower import piper_follower

    calls = []
    follower = object.__new__(piper_follower.PiperFollower)
    follower.bus = SimpleNamespace(
        is_connected=True,
        safe_disconnect=lambda: calls.append("safe_disconnect"),
        connect=lambda enable: (_ for _ in ()).throw(RuntimeError("disable failed")),
    )
    follower.cameras = {
        "camera": SimpleNamespace(
            is_connected=True,
            disconnect=lambda: calls.append("camera_disconnect"),
        )
    }
    monkeypatch.setattr(piper_follower.time, "sleep", lambda seconds: None)

    with pytest.raises(RuntimeError, match="disable failed"):
        follower.disconnect()

    assert calls == ["safe_disconnect", "camera_disconnect"]


def test_follower_connect_disables_bus_when_camera_cleanup_fails():
    from lerobot.robots.piper_follower import PiperFollower

    calls = []
    follower = object.__new__(PiperFollower)
    bus = SimpleNamespace(is_connected=False)

    def connect_bus(enable):
        calls.append(("bus_connect", enable))
        bus.is_connected = enable
        return True

    follower.bus = bus
    bus.connect = connect_bus
    follower.cameras = {
        "first": SimpleNamespace(
            is_connected=True,
            connect=lambda: (_ for _ in ()).throw(RuntimeError("camera connect failed")),
            disconnect=lambda: (_ for _ in ()).throw(RuntimeError("camera disconnect failed")),
        ),
        "second": SimpleNamespace(
            is_connected=True,
            connect=lambda: None,
            disconnect=lambda: calls.append("second_camera_disconnect"),
        ),
    }

    with pytest.raises(RuntimeError, match="camera connect failed"):
        follower.connect()

    assert calls == [("bus_connect", True), "second_camera_disconnect", ("bus_connect", False)]


def test_follower_send_action_uses_fixed_motor_order_when_config_is_unordered():
    from lerobot.robots.piper_follower import PiperFollower

    targets = []
    follower = object.__new__(PiperFollower)
    follower.bus = SimpleNamespace(
        is_connected=True,
        motors={
            "gripper": (7, "agilex_piper"),
            "joint_3": (3, "agilex_piper"),
            "joint_1": (1, "agilex_piper"),
            "joint_6": (6, "agilex_piper"),
            "joint_2": (2, "agilex_piper"),
            "joint_4": (4, "agilex_piper"),
            "joint_5": (5, "agilex_piper"),
        },
        write=lambda target: targets.append(target),
    )
    follower.cameras = {}
    action = {
        "joint_1.pos": 0.0,
        "joint_2.pos": 1.0,
        "joint_3.pos": 2.0,
        "joint_4.pos": 3.0,
        "joint_5.pos": 4.0,
        "joint_6.pos": 5.0,
        "gripper.pos": 6.0,
    }

    follower.send_action(action)

    assert targets == [[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]


def test_leader_features_use_fixed_motor_order_when_config_is_unordered():
    from lerobot.teleoperators.piper_leader import PiperLeader

    leader = object.__new__(PiperLeader)
    leader.bus = SimpleNamespace(
        motors={
            "gripper": (7, "agilex_piper"),
            "joint_3": (3, "agilex_piper"),
            "joint_1": (1, "agilex_piper"),
            "joint_6": (6, "agilex_piper"),
            "joint_2": (2, "agilex_piper"),
            "joint_4": (4, "agilex_piper"),
            "joint_5": (5, "agilex_piper"),
        }
    )

    assert list(leader.action_features) == [
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "gripper.pos",
    ]


def test_piper_follower_is_supported_by_async_client():
    """Piper must pass the async client robot-type gate without opening hardware."""
    from lerobot.async_inference.constants import SUPPORTED_ROBOTS

    assert "piper_follower" in SUPPORTED_ROBOTS


def test_piper_camera_features_build_async_observation_without_double_prefix():
    """Piper camera keys must match `get_observation` values before dataset conversion."""
    from lerobot.robots.piper_follower.piper_follower import PiperFollower
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features

    robot = object.__new__(PiperFollower)
    robot.cameras = {"pikaGripperDepthCamera": SimpleNamespace(height=480, width=640)}
    features = hw_to_dataset_features(robot._cameras_ft, OBS_STR, use_video=False)
    image = object()

    assert set(features) == {"observation.images.pikaGripperDepthCamera"}
    assert build_dataset_frame(features, {"pikaGripperDepthCamera": image}, OBS_STR) == {
        "observation.images.pikaGripperDepthCamera": image
    }


def test_piper_action_tensor_maps_to_joint_order_without_hardware():
    """The remote action vector must preserve Piper's expected joint order."""
    pytest.importorskip("grpc")
    import torch

    from lerobot.async_inference.robot_client import RobotClient

    client = object.__new__(RobotClient)
    client.robot = SimpleNamespace(
        action_features={
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "gripper.pos": float,
        }
    )

    assert client._action_tensor_to_action_dict(torch.arange(7, dtype=torch.float32)) == {
        "joint_1.pos": 0.0,
        "joint_2.pos": 1.0,
        "joint_3.pos": 2.0,
        "joint_4.pos": 3.0,
        "joint_5.pos": 4.0,
        "joint_6.pos": 5.0,
        "gripper.pos": 6.0,
    }
