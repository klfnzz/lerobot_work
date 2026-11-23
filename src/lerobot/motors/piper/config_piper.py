from dataclasses import dataclass, field
from typing import Literal

PIPER_MOTORS = {
    "joint_1": (1, "agilex_piper"),
    "joint_2": (2, "agilex_piper"),
    "joint_3": (3, "agilex_piper"),
    "joint_4": (4, "agilex_piper"),
    "joint_5": (5, "agilex_piper"),
    "joint_6": (6, "agilex_piper"),
    "gripper": (7, "agilex_piper"),
}
PIPER_MOTOR_NAMES = tuple(PIPER_MOTORS)


@dataclass
class PiperMotorsBusConfig:
    can_name: str
    motors: dict[str, tuple[int, str]] = field(default_factory=lambda: PIPER_MOTORS.copy())
    robot_model: Literal["piper", "piper_h", "piper_l", "piper_x"] = "piper"
    firmware_version: Literal["default", "v183", "v188"] = "default"
    interface: str = "socketcan"
    bitrate: int = 1_000_000
    gripper_force: float = 1.0
    enable_check_can: bool = True
