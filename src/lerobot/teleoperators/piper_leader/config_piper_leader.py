from dataclasses import dataclass, field

from lerobot.motors.piper.config_piper import PiperMotorsBusConfig
from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("piper_leader")
@dataclass
class PiperLeaderConfig(TeleoperatorConfig):
    motors: PiperMotorsBusConfig = field(default_factory=lambda: PiperMotorsBusConfig(can_name="can_master1"))
