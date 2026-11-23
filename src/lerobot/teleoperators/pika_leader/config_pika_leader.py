from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("pika_leader")
@dataclass
class PikaLeaderConfig(TeleoperatorConfig):
    """Configuration for the Pika handheld 6-DoF spatial teleoperator (via local ROS 2 IK bridge).

    Produces 7-D Piper joint-space actions (`joint_1.pos`..`joint_6.pos`, `gripper.pos`) anchored
    to the follower robot's current pose on engage, making it a drop-in teleoperator for
    `lerobot-teleoperate`, `lerobot-record`, and `lerobot-rollout --strategy.type=dagger`.
    """

    host: str = "127.0.0.1"
    port: int = 18765
    timeout_s: float = 1.0
    follower_can_name: str = "can0"
    engage_on_start: bool = True
    auto_reanchor_gap_s: float = 0.5
    require_pose_on_connect: bool = False
    fallback_joint_position: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not (1 <= self.port <= 65535):
            raise ValueError(f"pika_leader port must be in 1..65535, got {self.port}.")
        if self.timeout_s <= 0:
            raise ValueError(f"pika_leader timeout_s must be > 0, got {self.timeout_s}.")
        if self.auto_reanchor_gap_s < 0:
            raise ValueError(
                f"pika_leader auto_reanchor_gap_s must be >= 0, got {self.auto_reanchor_gap_s}."
            )
        if len(self.fallback_joint_position) != 7:
            raise ValueError(
                f"pika_leader fallback_joint_position must have 7 values (6 joints + gripper), "
                f"got {len(self.fallback_joint_position)}."
            )
