import math
import time
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.motors.piper import PiperMotorsBus, record_piper_state
from lerobot.motors.piper.config_piper import PIPER_MOTOR_NAMES
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_piper_follower import PiperFollowerConfig


class PiperFollower(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"
    # move_j and move_gripper_m use absolute position targets, so refreshing the
    # complete applied command retains the target while remote inference runs.
    supports_position_hold = True

    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.bus = PiperMotorsBus(config.motors)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in PIPER_MOTOR_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        features = {}
        for name, camera in self.cameras.items():
            if camera.height is None or camera.width is None:
                raise ValueError(f"Piper camera {name!r} requires explicit height and width.")
            features[name] = (camera.height, camera.width, 3)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        try:
            if not self.bus.connect(enable=True):
                raise ConnectionError("Failed to enable Piper follower.")
            for camera in self.cameras.values():
                camera.connect()
            if calibrate and not self.is_calibrated:
                self.calibrate()
        except Exception:
            try:
                for camera in self.cameras.values():
                    if camera.is_connected:
                        try:
                            camera.disconnect()
                        except Exception:
                            continue
            finally:
                self.bus.connect(enable=False)
            raise

    def calibrate(self) -> None:
        if not self.bus.is_connected:
            raise ConnectionError("Piper follower is not connected.")
        self.bus.apply_calibration()

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        raw_state = self.bus.read()
        can_name = getattr(getattr(getattr(self, "config", None), "motors", None), "can_name", None)
        record_piper_state(can_name, raw_state)
        observation: dict[str, Any] = {f"{joint}.pos": value for joint, value in raw_state.items()}
        for name, camera in self.cameras.items():
            observation[name] = camera.async_read()
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        applied_action = {f"{motor}.pos": float(action[f"{motor}.pos"]) for motor in PIPER_MOTOR_NAMES}
        if not all(math.isfinite(value) for value in applied_action.values()):
            raise ValueError("Piper position targets must be finite.")
        applied_action["gripper.pos"] = max(0.0, applied_action["gripper.pos"])
        self.bus.write(list(applied_action.values()))
        return applied_action

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            self.bus.safe_disconnect()
            time.sleep(5.0)
        finally:
            try:
                self.bus.connect(enable=False)
            finally:
                for camera in self.cameras.values():
                    if camera.is_connected:
                        camera.disconnect()
