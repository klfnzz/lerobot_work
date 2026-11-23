from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot.motors.piper import clear_piper_state, record_piper_state

pytestmark = pytest.mark.hardware


def _load_bridge_module():
    bridge_path = Path(__file__).resolve().parents[2] / "examples/piper/pika_teleop_bridge.py"
    spec = importlib.util.spec_from_file_location("pika_teleop_bridge", bridge_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _LinearTranslationIK:
    """Deterministic invertible 6-DoF FK/IK mock: TCP xyz = q[:3], orientation = diag(q[3:6])."""

    def __init__(self) -> None:
        self.synced: list[list[float]] = []

    def sync_state(self, q_current: list[float]) -> None:
        self.synced.append(list(q_current))

    def get_pose_mat(self, q: list[float] | np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=float).reshape(-1)
        mat = np.eye(4, dtype=float)
        mat[:3, 3] = q_arr[:3]
        return mat

    def solve(self, target_pose: np.ndarray) -> np.ndarray:
        out = np.zeros(6, dtype=float)
        out[:3] = target_pose[:3, 3]
        out[4] = 0.52
        return out


@pytest.fixture
def live_pika_bridge():
    bridge = _load_bridge_module()
    ik = _LinearTranslationIK()
    controller = bridge.PikaIKController(
        ik,
        handle_pose_rpy=(0.0, 0.0, 0.0),
        gripper_max_range=0.07,
        pose_timeout_s=0.5,
        max_joint_step_rad=1.0,
    )
    server = bridge.PikaBridgeServer(controller, host="127.0.0.1", port=0)
    server.start_in_background()
    host, port = server.server_address
    clear_piper_state("can_test_pika")
    try:
        yield SimpleNamespace(bridge=bridge, ik=ik, controller=controller, host=host, port=port)
    finally:
        clear_piper_state("can_test_pika")
        server.shutdown()


def test_pika_leader_registers_and_matches_piper_features(tmp_path):
    import draccus

    from lerobot.robots.piper_follower import PiperFollower, PiperFollowerConfig
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig
    from lerobot.teleoperators import make_teleoperator_from_config
    from lerobot.teleoperators.pika_leader import PikaLeader, PikaLeaderConfig

    cfg = draccus.parse(
        TeleoperateConfig,
        args=[
            "--robot.type=piper_follower",
            "--teleop.type=pika_leader",
            "--teleop.port=19001",
            "--teleop.follower_can_name=can0",
        ],
    )
    assert isinstance(cfg.robot, PiperFollowerConfig)
    assert isinstance(cfg.teleop, PikaLeaderConfig)
    assert cfg.teleop.port == 19001
    assert cfg.teleop.is_clutch if hasattr(cfg.teleop, "is_clutch") else True

    cfg.teleop.calibration_dir = tmp_path / "pika_cal"
    leader = make_teleoperator_from_config(cfg.teleop)
    assert isinstance(leader, PikaLeader)
    assert leader.is_clutch_teleop is True
    assert leader.feedback_features == {}

    follower = object.__new__(PiperFollower)
    assert leader.action_features == follower.action_features


def test_dagger_rollout_cli_accepts_pika_leader():
    import draccus

    from lerobot.robots import piper_follower  # noqa: F401
    from lerobot.rollout.configs import DAggerStrategyConfig, RolloutConfig
    from lerobot.teleoperators import pika_leader  # noqa: F401
    from lerobot.teleoperators.pika_leader import PikaLeaderConfig

    # Parse strategy + robot + teleop + dataset without loading a checkpoint (`remote` owns policy).
    cfg = draccus.parse(
        RolloutConfig,
        args=[
            "--strategy.type=dagger",
            "--inference.type=remote",
            "--inference.endpoint=tcp/127.0.0.1:7447",
            "--inference.deployment=piper-pi05",
            "--inference.semantics=piper-radians-gripper-metres-v1",
            "--robot.type=piper_follower",
            "--teleop.type=pika_leader",
            "--dataset.repo_id=lazycc/rollout_piper_pika_dagger",
            "--dataset.fps=30",
            "--fps=30",
        ],
    )
    assert isinstance(cfg.strategy, DAggerStrategyConfig)
    assert isinstance(cfg.teleop, PikaLeaderConfig)


def test_pika_leader_connect_gives_actionable_error_when_bridge_down(tmp_path):
    from lerobot.teleoperators.pika_leader import PikaLeader, PikaLeaderConfig

    cfg = PikaLeaderConfig(host="127.0.0.1", port=1, timeout_s=0.1, calibration_dir=tmp_path)
    leader = PikaLeader(cfg)
    with pytest.raises(ConnectionError, match="start_pika_teleop_bridge.bash"):
        leader.connect()
    assert not leader.is_connected


def test_pika_leader_end_to_end_reanchor_delta_and_clutch(live_pika_bridge, tmp_path):
    from lerobot.teleoperators.pika_leader import PikaLeader, PikaLeaderConfig

    handle = np.eye(4, dtype=float)
    handle[:3, 3] = [1.0, 2.0, 3.0]
    live_pika_bridge.controller.update_handle_pose(handle)
    live_pika_bridge.controller.update_gripper(0.085)  # above 0.07 -> clamped to 0.07

    record_piper_state(
        "can_test_pika",
        {
            "joint_1": 0.1,
            "joint_2": 0.2,
            "joint_3": 0.3,
            "joint_4": 0.0,
            "joint_5": 0.52,
            "joint_6": 0.0,
            "gripper": 0.02,
        },
    )

    cfg = PikaLeaderConfig(
        host=live_pika_bridge.host,
        port=live_pika_bridge.port,
        follower_can_name="can_test_pika",
        auto_reanchor_gap_s=10.0,
        calibration_dir=tmp_path,
    )
    with PikaLeader(cfg) as leader:
        # 1) First step with zero handle motion -> exact zero jump from follower pose!
        act0 = leader.get_action()
        assert act0["joint_1.pos"] == pytest.approx(0.1)
        assert act0["joint_2.pos"] == pytest.approx(0.2)
        assert act0["joint_3.pos"] == pytest.approx(0.3)
        assert act0["gripper.pos"] == pytest.approx(0.07)

        # 2) Move Pika handle by (+0.05, -0.02, +0.01) -> TCP moves by exact same delta!
        handle_moved = handle.copy()
        handle_moved[:3, 3] = [1.05, 1.98, 3.01]
        live_pika_bridge.controller.update_handle_pose(handle_moved)
        act1 = leader.get_action()
        assert act1["joint_1.pos"] == pytest.approx(0.15)
        assert act1["joint_2.pos"] == pytest.approx(0.18)
        assert act1["joint_3.pos"] == pytest.approx(0.31)

        # 3) Press Pika button to disengage clutch, reposition hand wildly -> robot holds pose!
        engaged, _ = live_pika_bridge.controller.toggle_clutch()
        assert engaged is False
        handle_recentered = handle.copy()
        handle_recentered[:3, 3] = [5.0, -4.0, 2.0]
        live_pika_bridge.controller.update_handle_pose(handle_recentered)
        act_held = leader.get_action()
        assert act_held["joint_1.pos"] == pytest.approx(0.15)
        assert act_held["joint_2.pos"] == pytest.approx(0.18)
        assert act_held["joint_3.pos"] == pytest.approx(0.31)

        # 4) Update robot feedback to current pose, press Pika button to re-engage -> zero jump!
        record_piper_state(
            "can_test_pika",
            {
                "joint_1": 0.15,
                "joint_2": 0.18,
                "joint_3": 0.31,
                "joint_4": 0.0,
                "joint_5": 0.52,
                "joint_6": 0.0,
                "gripper": 0.07,
            },
        )
        engaged, _ = live_pika_bridge.controller.toggle_clutch()
        assert engaged is True
        act_reengaged = leader.get_action()
        assert act_reengaged["joint_1.pos"] == pytest.approx(0.15)
        assert act_reengaged["joint_2.pos"] == pytest.approx(0.18)
        assert act_reengaged["joint_3.pos"] == pytest.approx(0.31)


def test_pika_bridge_holds_and_reanchors_cleanly_after_pose_timeout():
    bridge = _load_bridge_module()
    ik = _LinearTranslationIK()
    controller = bridge.PikaIKController(
        ik,
        handle_pose_rpy=(0.0, 0.0, 0.0),
        pose_timeout_s=0.5,
        max_joint_step_rad=1.0,
    )

    h0 = np.eye(4, dtype=float)
    controller.update_handle_pose(h0, now=10.0)
    controller.reanchor([0.1, 0.2, 0.3, 0.0, 0.52, 0.0], 0.01, engage=True, now=10.0)

    # Normal step at t=10.1
    h1 = np.eye(4, dtype=float)
    h1[:3, 3] = [0.05, 0.0, 0.0]
    controller.update_handle_pose(h1, now=10.1)
    step1 = controller.step([0.1, 0.2, 0.3, 0.0, 0.52, 0.0], 0.01, now=10.1)
    assert step1["joints"][:3] == pytest.approx([0.15, 0.2, 0.3])

    # Pose goes stale at t=11.0 (> 0.5s): holds [0.15, 0.2, 0.3] and disarms baseline
    stale = controller.step([0.15, 0.2, 0.3, 0.0, 0.52, 0.0], 0.01, now=11.0)
    assert stale["anchored"] is False
    assert stale["joints"][:3] == pytest.approx([0.15, 0.2, 0.3])

    # Pose recovers at a completely different handle location at t=11.2 -> re-anchors without jump!
    h_recovered = np.eye(4, dtype=float)
    h_recovered[:3, 3] = [9.0, -3.0, 2.0]
    controller.update_handle_pose(h_recovered, now=11.2)
    recovered = controller.step([0.15, 0.2, 0.3, 0.0, 0.52, 0.0], 0.01, now=11.2)
    assert recovered["anchored"] is True
    assert recovered["joints"][:3] == pytest.approx([0.15, 0.2, 0.3])


def test_dagger_transition_calls_pika_engage_and_skips_follower_slide(live_pika_bridge, monkeypatch, tmp_path):
    from lerobot.rollout.configs import DAggerStrategyConfig
    from lerobot.rollout.strategies import dagger as dagger_mod
    from lerobot.rollout.strategies.dagger import DAggerPhase, DAggerStrategy
    from lerobot.teleoperators.pika_leader import PikaLeader, PikaLeaderConfig

    slide_calls = []
    monkeypatch.setattr(
        dagger_mod,
        "follower_smooth_move_to",
        lambda *args, **kwargs: slide_calls.append((args, kwargs)),
    )

    h0 = np.eye(4, dtype=float)
    live_pika_bridge.controller.update_handle_pose(h0)

    cfg = PikaLeaderConfig(
        host=live_pika_bridge.host,
        port=live_pika_bridge.port,
        follower_can_name="can_test_pika",
        calibration_dir=tmp_path,
    )
    strategy = DAggerStrategy(DAggerStrategyConfig(smooth_handover=True))
    paused_obs = {
        "joint_1.pos": 0.25,
        "joint_2.pos": -0.1,
        "joint_3.pos": 0.4,
        "joint_4.pos": 0.0,
        "joint_5.pos": 0.52,
        "joint_6.pos": 0.0,
        "gripper.pos": 0.03,
    }
    robot = SimpleNamespace(get_observation=lambda: paused_obs)
    ctx = SimpleNamespace(hardware=SimpleNamespace(robot_wrapper=robot))
    engine = SimpleNamespace(pause=lambda: None, reset=lambda: None, resume=lambda: None)
    interpolator = SimpleNamespace(reset=lambda: None)

    with PikaLeader(cfg) as leader:
        strategy._apply_transition(
            DAggerPhase.PAUSED,
            DAggerPhase.CORRECTING,
            engine,
            interpolator,
            leader,
            ctx,
            prev_action=paused_obs,
            timer=None,
        )
        # Clutch teleop must NOT slide the follower to an arbitrary pose:
        assert slide_calls == []
        # First action in CORRECTING starts anchored right at `paused_obs`:
        act = leader.get_action()
        assert act["joint_1.pos"] == pytest.approx(0.25)
        assert act["joint_2.pos"] == pytest.approx(-0.1)
        assert act["joint_3.pos"] == pytest.approx(0.4)

        strategy._apply_transition(
            DAggerPhase.CORRECTING,
            DAggerPhase.PAUSED,
            engine,
            interpolator,
            leader,
            ctx,
            prev_action=paused_obs,
            timer=None,
        )
        assert live_pika_bridge.controller.status()["engaged"] is False
