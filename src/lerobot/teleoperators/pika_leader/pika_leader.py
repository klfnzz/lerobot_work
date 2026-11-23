import json
import logging
import math
import socket
import threading
import time
from typing import Any

from lerobot.lerobot_types import RobotAction
from lerobot.motors.piper import get_latest_piper_state
from lerobot.motors.piper.config_piper import PIPER_MOTOR_NAMES
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_pika_leader import PikaLeaderConfig

logger = logging.getLogger(__name__)


class PikaLeader(Teleoperator):
    """Pika handheld spatial teleoperator producing Piper 7-D joint actions via local ROS 2 IK bridge.

    As a clutch-style 6-DoF device (`is_clutch_teleop = True`), Pika re-references its handle frame
    against the follower arm's live joint pose whenever teleoperation engages (startup, DAgger
    `PAUSED -> CORRECTING`, episode reset, Pika handle button toggle, or after a control gap).
    """

    config_class = PikaLeaderConfig
    name = "pika_leader"
    is_clutch_teleop = True

    def __init__(self, config: PikaLeaderConfig):
        super().__init__(config)
        self.config = config
        self._sock: socket.socket | None = None
        self._reader: Any = None
        self._io_lock = threading.Lock()
        self._is_connected = False
        self._is_calibrated = False
        self._needs_reanchor = True
        self._force_engage_next = False
        self._first_reanchor_done = False
        self._last_action_ts: float | None = None
        self._last_targets: list[float] = [float(v) for v in config.fallback_joint_position]

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in PIPER_MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def _open_socket(self) -> None:
        sock = socket.create_connection((self.config.host, self.config.port), timeout=self.config.timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.config.timeout_s)
        self._sock = sock
        self._reader = sock.makefile("r", encoding="utf-8")

    def _close_socket(self) -> None:
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:  # nosec B110
                pass
            self._reader = None
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:  # nosec B110
                pass
            self._sock = None

    def _rpc(self, payload: dict[str, Any], *, allow_reconnect: bool = True) -> dict[str, Any]:
        message = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        with self._io_lock:
            for attempt in range(2 if allow_reconnect else 1):
                try:
                    if self._sock is None or self._reader is None:
                        self._open_socket()
                    assert self._sock is not None and self._reader is not None
                    self._sock.sendall(message)
                    line = self._reader.readline()
                    if not line:
                        raise ConnectionError("Pika teleop bridge closed the connection.")
                    response = json.loads(line)
                    if not isinstance(response, dict):
                        raise ValueError(f"Unexpected Pika bridge response: {response!r}")
                    if not response.get("ok", False):
                        raise RuntimeError(f"Pika bridge error: {response.get('error', response)}")
                    return response
                except (OSError, ConnectionError) as exc:
                    self._close_socket()
                    if attempt == 0 and allow_reconnect:
                        continue
                    raise ConnectionError(
                        f"Cannot communicate with Pika teleop bridge at {self.config.host}:{self.config.port}. "
                        "Start it in another terminal first:\n"
                        "  bash examples/piper/start_pika_teleop_bridge.bash"
                    ) from exc
        raise RuntimeError("Unreachable RPC loop termination")

    def _extract_follower_targets(self, obs: dict[str, Any] | None = None) -> list[float]:
        if obs is not None:
            try:
                return [float(obs[f"{motor}.pos"]) for motor in PIPER_MOTOR_NAMES]
            except KeyError:
                pass

        cached = get_latest_piper_state(self.config.follower_can_name)
        if cached is not None:
            _ts, state = cached
            return [float(state[motor]) for motor in PIPER_MOTOR_NAMES]

        return list(self._last_targets)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        try:
            status = self._rpc({"cmd": "ping"}, allow_reconnect=False)
            if self.config.require_pose_on_connect and not status.get("has_pose", False):
                raise ConnectionError(
                    "Pika teleop bridge is reachable, but no fresh /pika_pose has been received yet. "
                    "Verify the Vive base station and pika_single_locator_node are running."
                )
            self._is_connected = True
            self._needs_reanchor = True
            self._first_reanchor_done = False
            self._last_action_ts = None
            if calibrate and not self._is_calibrated:
                self.calibrate()
            logger.info(
                "Connected to Pika teleop bridge at %s:%d (has_pose=%s, has_gripper=%s)",
                self.config.host,
                self.config.port,
                status.get("has_pose"),
                status.get("has_gripper"),
            )
        except Exception:
            self._is_connected = False
            self._is_calibrated = False
            self._close_socket()
            raise

    def calibrate(self) -> None:
        if not self._is_connected:
            raise ConnectionError("Pika leader is not connected.")
        self._needs_reanchor = True
        self._is_calibrated = True

    def configure(self) -> None:
        pass

    def prepare_engage(self, hint_state: dict[str, Any] | None = None) -> None:
        """Arm an immediate re-anchor for the next `get_action()` call without extra hardware reads."""
        if hint_state is not None:
            try:
                self._last_targets = [float(hint_state[f"{motor}.pos"]) for motor in PIPER_MOTOR_NAMES]
            except KeyError:
                pass
        self._needs_reanchor = True
        self._force_engage_next = True
        self._last_action_ts = None

    def on_engage(self, obs: dict[str, Any] | None = None) -> None:
        """Explicitly anchor Pika to the follower's current pose when entering teleop/correction."""
        if not self._is_connected:
            return
        targets = self._extract_follower_targets(obs)
        self._rpc(
            {
                "cmd": "reanchor",
                "joints": targets[:6],
                "gripper": max(0.0, targets[6]),
                "engage": True,
            }
        )
        self._last_targets = targets
        self._needs_reanchor = False
        self._force_engage_next = False
        self._first_reanchor_done = True
        self._last_action_ts = time.monotonic()

    def on_disengage(self) -> None:
        """Mark teleop disengaged on correction/reset exit so the next engage always re-anchors."""
        self._needs_reanchor = True
        self._force_engage_next = False
        self._last_action_ts = None
        if not self._is_connected:
            return
        try:
            self._rpc({"cmd": "disengage"}, allow_reconnect=False)
        except Exception:
            logger.debug("Ignoring bridge disengage error during transition", exc_info=True)

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        now = time.monotonic()
        targets = self._extract_follower_targets()
        gap_exceeded = (
            self._last_action_ts is not None
            and self.config.auto_reanchor_gap_s > 0
            and (now - self._last_action_ts) > self.config.auto_reanchor_gap_s
        )
        if self._needs_reanchor or gap_exceeded:
            engage = (
                True
                if (self._force_engage_next or self._first_reanchor_done)
                else self.config.engage_on_start
            )
            self._rpc(
                {
                    "cmd": "reanchor",
                    "joints": targets[:6],
                    "gripper": max(0.0, targets[6]),
                    "engage": engage,
                }
            )
            self._needs_reanchor = False
            self._force_engage_next = False
            self._first_reanchor_done = True

        reply = self._rpc(
            {
                "cmd": "step",
                "joints": targets[:6],
                "gripper": max(0.0, targets[6]),
            }
        )
        joints = reply.get("joints")
        gripper = reply.get("gripper")
        if not isinstance(joints, list) or len(joints) != 6 or gripper is None:
            raise ValueError(f"Invalid step payload from Pika bridge: {reply!r}")

        combined = [*(float(v) for v in joints), max(0.0, float(gripper))]
        if not all(math.isfinite(v) for v in combined):
            raise ValueError(f"Non-finite target received from Pika bridge: {combined!r}")

        self._last_targets = combined
        self._last_action_ts = time.monotonic()
        return dict(zip((f"{m}.pos" for m in PIPER_MOTOR_NAMES), combined, strict=True))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            self.on_disengage()
        finally:
            self._is_connected = False
            self._is_calibrated = False
            self._close_socket()
