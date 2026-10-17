from lerobot.lerobot_types import RobotAction
from lerobot.motors.piper import PiperMotorsBus
from lerobot.motors.piper.config_piper import PIPER_MOTOR_NAMES
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_piper_leader import PiperLeaderConfig


class PiperLeader(Teleoperator):
    config_class = PiperLeaderConfig
    name = "piper_leader"

    def __init__(self, config: PiperLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = PiperMotorsBus(config.motors)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in PIPER_MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        try:
            if not self.bus.connect(enable=True):
                raise ConnectionError("Failed to enable Piper leader.")
            if calibrate and not self.is_calibrated:
                self.calibrate()
        except Exception:
            self.bus.connect(enable=False)
            raise

    def calibrate(self) -> None:
        if not self.bus.is_connected:
            raise ConnectionError("Piper leader is not connected.")
        self.bus.apply_calibration()

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        return {f"{joint}.pos": value for joint, value in self.bus.read().items()}

    def send_feedback(self, feedback: dict) -> None:
        return None

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            self.bus.safe_disconnect()
        finally:
            self.bus.connect(enable=False)
