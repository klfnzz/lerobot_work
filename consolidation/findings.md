# Consolidation findings

- User's lerobot_ronot_piper means existing lerobot_robot_piper.
- 061 is official 0.6.1 checkout at prior release commit 7e241bd, with its own Git repository; only previous dataset task records are untracked.
- lerobot_piper3 is older 0.4.4, a root Git submodule, with uncommitted configuration/async/training changes and command.sh.
- lerobot_robot_piper is a standalone plugin ported to 0.6.1 with tested safety cleanup; use it as the main Piper implementation.
- Existing 061 environment was named lerobot_piper061, Python 3.12. Previous full suite hit absent Git LFS test images.
- Existing root planning files describe May GitHub migration; old checkout records cover multiple tasks. This task uses this separate consolidation directory.

- Native integration uses the plugin's tested lifecycle behavior and 0.6.1's existing config-to-device factory; CLI registration imports follow other built-in robots. No compatibility wrapper/plugin package remains necessary.
- Source settings: follower can0, leader can_master1; camera serials 230322272075/233622078698 at 640x480/30 FPS; socketcan 1 Mbps, force 1.0, fixed joint_1..6/gripper order and rad/m units.
- Old outputs total 9.4 GB; move intact, do not put model weights into source archive. Old command.sh is mixed prose/commands, to preserve as historical notes, not executable.
- Old global training preferences (batch_size=64, ACT chunk/action_steps=30, diffusion clip_sample=false) belong in Piper command examples; retain 061 core defaults.
- Legacy leader had relative calibration_dir=.cache/calibration/piper_leader; plugin/061 uses standard cache location (None in config). Preserve legacy override in migration usage notes, keep 061 default. No local legacy calibration cache found.

- Review found generic find-joint-limits assumes degree-based kinematics; Piper emits radians. Removed the newly added Piper imports from that tool and setup-motors (unsupported interface), restoring both files exactly to upstream. Two subprocess regression tests reproduced unwanted CLI advertising before correction.
- Existing uv.lock refreshed using local uv; all preexisting package versions/sources unchanged. Only pyagxarm 1.0.0 at the already-used SDK revision added; lock format/order updated by uv.
- Single active checkout verified: /home/qwe/lerobot_exp/lerobot_piper061. Old standalone plugin distribution removed; old 0.4.4 editable import removed; 061 imports directly from native source and reports 0.6.1.
- Durable archives: consolidation/archives/lerobot_piper3-20261017.tar.gz and lerobot_robot_piper-20261017.tar.gz. Temporary retired source copies additionally remain at /tmp/lerobot-consolidation-retired-20261017; durable archives do not depend on /tmp.
