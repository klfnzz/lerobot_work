from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.motors.piper.config_piper import PiperMotorsBusConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("piper_follower")
@dataclass
class PiperFollowerConfig(RobotConfig):
    motors: PiperMotorsBusConfig = field(default_factory=lambda: PiperMotorsBusConfig(can_name="can0"))
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.cameras:
            self.cameras = {
                "pikaGripperDepthCamera": RealSenseCameraConfig(
                    serial_number_or_name="230322272075", fps=30, width=640, height=480
                ),
                "highDepthCamera": RealSenseCameraConfig(
                    serial_number_or_name="233622078698", fps=30, width=640, height=480
                ),
                "sideDepthCamera": RealSenseCameraConfig(
                    serial_number_or_name="109122072082", fps=30, width=640, height=480
                ),
            }
        super().__post_init__()
