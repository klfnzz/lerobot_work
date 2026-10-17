import time
from typing import TYPE_CHECKING, Any

from lerobot.utils.import_utils import _pyagxarm_available, require_package

from .config_piper import PiperMotorsBusConfig

if TYPE_CHECKING or _pyagxarm_available:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config


class PiperMotorsBus:
    def __init__(self, config: PiperMotorsBusConfig):
        require_package("pyAgxArm", "piper")
        self.config = config
        self.motors = config.motors
        self.init_joint_position = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0]
        self.safe_disable_position = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0]
        self._is_connected = False
        self._is_calibrated = False

        arm_config = create_agx_arm_config(
            robot=config.robot_model,
            firmeware_version=config.firmware_version,
            interface=config.interface,
            channel=config.can_name,
            bitrate=config.bitrate,
            enable_check_can=config.enable_check_can,
            auto_connect=False,
        )
        self.piper = AgxArmFactory.create_arm(arm_config)
        self.gripper = self.piper.init_effector(self.piper.OPTIONS.EFFECTOR.AGX_GRIPPER)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def _set_enable_state(self, enable: bool, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() <= deadline:
            if enable:
                self.piper.enable()
            else:
                self.piper.disable()

            statuses = self.piper.get_joints_enable_status_list()
            if statuses and all(bool(status) == enable for status in statuses):
                return True
            time.sleep(0.1)
        return False

    def connect(self, enable: bool) -> bool:
        if enable:
            if not self.piper.is_connected():
                self.piper.connect()
            self._is_connected = self._set_enable_state(True)
            if self._is_connected:
                self.gripper.move_gripper_m(value=0.0, force=self.config.gripper_force)
            return self._is_connected

        if self.piper.is_connected():
            self.gripper.disable_gripper()
            disabled = self._set_enable_state(False)
            self.piper.disconnect()
        else:
            disabled = True

        self._is_connected = False
        self._is_calibrated = False
        return disabled

    def apply_calibration(self) -> None:
        self.write(self.init_joint_position)
        self._is_calibrated = True

    def write(self, target_joint: list[float]) -> None:
        if len(target_joint) != 7:
            raise ValueError(f"Expected 7 Piper targets, got {len(target_joint)}.")
        if not self.piper.is_connected():
            raise ConnectionError("Piper arm is not connected.")

        self.piper.move_j([float(value) for value in target_joint[:6]])
        self.gripper.move_gripper_m(value=max(0.0, float(target_joint[6])), force=self.config.gripper_force)

    def read(self) -> dict[str, float]:
        joint_msg = self.piper.get_joint_angles()
        if joint_msg is None:
            raise RuntimeError("No Piper joint feedback has been received yet.")

        state = {f"joint_{index}": float(value) for index, value in enumerate(joint_msg.msg, start=1)}
        state["gripper"] = 0.0
        gripper_msg: Any = self.gripper.get_gripper_status()
        if gripper_msg is not None and getattr(gripper_msg.msg, "mode", "width") == "width":
            state["gripper"] = float(gripper_msg.msg.value)
        return state

    def safe_disconnect(self) -> None:
        self.write(self.safe_disable_position)
