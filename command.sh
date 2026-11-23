# LeRobot 0.7.0 / Piper 命令笔记：按需复制对应段落到终端执行。

############################ 环境 ############################
source /home/qwe/lerobot/activate_lerobot.bash
cd /home/qwe/lerobot/lerobot_piper

export HF_USER=lazycc
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export HF_ENDPOINT=https://hf-mirror.com

# 已有 ACT、Pi0.5 模型训练时使用的双相机配置。
# 新采集及 30 Hz 远程模型沿用驱动中的当前三相机配置。
export PIPER_MODEL_CAMERAS='{
  pikaGripperDepthCamera: {type: intelrealsense, serial_number_or_name: "230322272075", fps: 30, width: 640, height: 480},
  sideDepthCamera: {type: intelrealsense, serial_number_or_name: "233622078698", fps: 30, width: 640, height: 480}
}'

# 安装或更新本地依赖时执行。
uv pip install --python "$CONDA_PREFIX/bin/python" \
  -e '.[piper,core_scripts,remote,training,pi,diffusion,pusht]' 'httpx[socks]'


############################ ACT 训练 ############################
# 使用下面采集命令生成的数据；已有数据时替换 repo_id 和 root。
lerobot-train \
  --dataset.repo_id=${HF_USER}/piper_pick_and_place \
  --dataset.root=outputs/datasets/piper_pick_and_place \
  --policy.type=act \
  --policy.chunk_size=30 \
  --policy.n_action_steps=30 \
  --batch_size=64 \
  --output_dir=outputs/train/act_pick_and_place \
  --job_name=act_pick_and_place \
  --policy.device=cuda \
  --trackio.enable=true \
  --policy.push_to_hub=false \
  --policy.repo_id=${HF_USER}/act_pick_and_place


############################ 从 checkpoint 续训 ############################
# config_path 指向要恢复的实际 checkpoint。
lerobot-train \
  --config_path=outputs/train/dp_pusht/checkpoints/last/pretrained_model/train_config.json \
  --resume=true


############################ Diffusion / PushT ############################
hf download lerobot/pusht \
  --repo-type dataset \
  --revision v3.0 \
  --local-dir "$HOME/.cache/huggingface/lerobot/lerobot/pusht"

lerobot-train \
  --dataset.repo_id=lerobot/pusht \
  --policy.type=diffusion \
  --policy.clip_sample=false \
  --batch_size=64 \
  --output_dir=outputs/train/dp_pusht \
  --job_name=dp_pusht \
  --policy.device=cuda \
  --trackio.enable=true \
  --policy.push_to_hub=false \
  --policy.repo_id=${HF_USER}/dp_pusht

lerobot-eval \
  --policy.path=outputs/train/dp_pusht/checkpoints/last/pretrained_model \
  --env.type=pusht \
  --eval.batch_size=10 \
  --eval.n_episodes=50 \
  --policy.device=cuda \
  --output_dir=outputs/eval/pusht


############################ 遥操作与采集 ############################
lerobot-teleoperate \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --teleop.type=piper_leader \
  --teleop.id=leader \
  --teleop.motors.can_name=can_master1 \
  --fps=30

lerobot-record \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --teleop.type=piper_leader \
  --teleop.id=leader \
  --teleop.motors.can_name=can_master1 \
  --dataset.repo_id=${HF_USER}/piper_pick_and_place \
  --dataset.root=outputs/datasets/piper_pick_and_place \
  --dataset.single_task="pick up the object and place it in the container" \
  --dataset.fps=30 \
  --dataset.episode_time_s=30 \
  --dataset.num_episodes=50 \
  --dataset.push_to_hub=false

lerobot-replay \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --dataset.repo_id=${HF_USER}/piper_pick_and_place \
  --dataset.root=outputs/datasets/piper_pick_and_place \
  --dataset.episode=0


############################ Pika 遥操作与采集 ############################
# 终端 1：在 pika (ROS 2 Humble) 环境中启动 Vive 定位、Pika 夹爪串口节点与 IK 桥接服务
# （不占用 can0 与 RealSense 相机；如果传感器节点已在其他终端启动，追加 --bridge-only）
bash examples/piper/start_pika_teleop_bridge.bash

# 终端 2：先执行上面的“环境”段落，再使用 --teleop.type=pika_leader 遥操或录制
# 操作中按一次 Pika 手柄按键可松开离合调整手部位置，再按一次重新锚定当前机械臂位姿继续控制
lerobot-teleoperate \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --teleop.type=pika_leader \
  --teleop.id=pika \
  --fps=30

lerobot-record \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --teleop.type=pika_leader \
  --teleop.id=pika \
  --dataset.repo_id=${HF_USER}/piper_pick_and_place \
  --dataset.root=outputs/datasets/piper_pick_and_place \
  --dataset.single_task="pick up the object and place it in the container" \
  --dataset.fps=30 \
  --dataset.episode_time_s=30 \
  --dataset.num_episodes=50 \
  --dataset.push_to_hub=false


############################ 下载训练好的模型 ############################
hf download lazycc/act_pick_and_place \
  --repo-type model \
  --local-dir outputs/models/act_pick_and_place

hf download lazycc/pi05_pick_and_place \
  --repo-type model \
  --local-dir outputs/models/lazycc/pi05_pick_and_place


############################ ACT 本地推理 ############################
lerobot-rollout \
  --strategy.type=base \
  --inference.type=sync \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --policy.path=outputs/models/act_pick_and_place/checkpoints/006000/pretrained_model \
  --policy.n_action_steps=30 \
  --task="Pick up the object" \
  --fps=10 \
  --device=cuda

# ACT 003000：时序集成，并保存一个评估 episode。
lerobot-rollout \
  --strategy.type=episodic \
  --inference.type=sync \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --policy.path=outputs/models/act_pick_and_place/checkpoints/003000/pretrained_model \
  --policy.n_action_steps=1 \
  --policy.temporal_ensemble_coeff=0.02 \
  --dataset.repo_id=${HF_USER}/eval_act_pick_and_place_003000 \
  --dataset.root=outputs/eval_act_pick_and_place_003000 \
  --dataset.single_task="Pick up the object" \
  --dataset.fps=10 \
  --dataset.episode_time_s=30 \
  --dataset.num_episodes=1 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --task="Pick up the object" \
  --fps=10 \
  --device=cuda

# ACT 006000：每次执行 30 个动作，并保存一个评估 episode。
# 已有同名评估目录时，更换 dataset.root，或追加 --resume=true 继续采集。
lerobot-rollout \
  --strategy.type=episodic \
  --inference.type=sync \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --policy.path=outputs/models/act_pick_and_place/checkpoints/006000/pretrained_model \
  --policy.n_action_steps=30 \
  --dataset.repo_id=${HF_USER}/eval_act_pick_and_place_006000 \
  --dataset.root=outputs/eval_act_pick_and_place_006000 \
  --dataset.single_task="Pick up the object" \
  --dataset.fps=10 \
  --dataset.episode_time_s=100 \
  --dataset.num_episodes=1 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --task="Pick up the object" \
  --fps=10 \
  --device=cuda


############################ Pi0.5 本地推理 ############################
# 准备 Piper 动作名称元数据；权重与 processor 链接到原文件。
python examples/piper/prepare_checkpoint.py \
  --source=outputs/models/lazycc/pi05_pick_and_place \
  --destination=outputs/models/lazycc/pi05_pick_and_place_piper

lerobot-rollout \
  --strategy.type=base \
  --inference.type=sync \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --policy.path=outputs/models/lazycc/pi05_pick_and_place_piper \
  --policy.compile_model=false \
  --task="pick up the object and place it in the container" \
  --fps=10 \
  --device=cuda

# Pi0.5 本地 RTC：在同一机器的后台线程运行推理。
lerobot-rollout \
  --strategy.type=base \
  --inference.type=rtc \
  --inference.rtc.enabled=true \
  --inference.rtc.mode=guided \
  --inference.rtc.execution_horizon=10 \
  --inference.rtc.max_guidance_weight=10 \
  --inference.queue_threshold=10 \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --policy.path=outputs/models/lazycc/pi05_pick_and_place_piper \
  --policy.compile_model=false \
  --task="pick up the object and place it in the container" \
  --fps=10 \
  --device=cuda


############################ 远程推理：GPU 服务端 ############################
# 远端进入同版本源码目录并激活 Python 3.12 环境后安装依赖。
uv pip install --python "$CONDA_PREFIX/bin/python" -e '.[remote,core_scripts,pi]' 'httpx[socks]'

# 以下服务端按模型选择一组运行，监听本机 7447 端口。

# Pi0.5 / 020000 / 10 Hz / 双相机。
python examples/piper/prepare_checkpoint.py \
  --source=/home/manager/wd/pi_med/lerobot/outputs/train/pi05_pick_place/checkpoints/020000/pretrained_model \
  --destination=outputs/models/piper-pi05-10hz

lerobot-policy-server \
  --config_path=examples/piper/remote_pi05_10hz.yaml \
  --deployment=piper-pi05-10hz \
  --model.repo_or_path=outputs/models/piper-pi05-10hz \
  --model.device=cuda \
  --model.compile_model=false \
  --execution.action_fps=10

# Pi0.5 / 030000 / 30 Hz / 当前三相机。
python examples/piper/prepare_checkpoint.py \
  --source=/home/manager/wd/pi_med/lerobot/outputs/train/muti_pick_and_place_task_single_arm_lerobot_30hz/checkpoints/030000/pretrained_model \
  --destination=outputs/models/piper-pi05

lerobot-policy-server \
  --config_path=examples/piper/remote_pi05.yaml \
  --deployment=piper-pi05 \
  --model.repo_or_path=outputs/models/piper-pi05 \
  --model.device=cuda \
  --model.compile_model=false \
  --execution.action_fps=30

# Pi0.5 插插头模型 / 040000 / 30 Hz / 当前三相机。
python examples/piper/prepare_checkpoint.py \
  --source=/home/manager/wd/pi_med/lerobot/outputs/train/plug_and_unplug_task_raw_30hz/checkpoints/040000/pretrained_model \
  --destination=outputs/models/piper-pi05-plug

lerobot-policy-server \
  --config_path=examples/piper/remote_pi05.yaml \
  --deployment=piper-pi05-plug \
  --model.repo_or_path=outputs/models/piper-pi05-plug \
  --model.device=cuda \
  --model.compile_model=false \
  --execution.action_fps=30


############################ 远程推理：本地 Piper 客户端 ############################
# 单独一个本地终端保持 SSH 隧道运行。
ssh -N -L 7447:127.0.0.1:7447 manager@172.29.220.198

# 另一个本地终端先执行上面的“环境”段落，再选择匹配的客户端。

# Pi0.5 / 020000 / 10 Hz。
lerobot-rollout \
  --strategy.type=base \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=piper-pi05-10hz \
  --inference.mode=chunk \
  --inference.semantics=piper-radians-gripper-metres-v1 \
  --inference.chunk_merge=aligned \
  --inference.refill_seconds=1.0 \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --task="pick up the object and place it in the container" \
  --fps=10

# Pi0.5 / 030000 / 30 Hz。
lerobot-rollout \
  --strategy.type=base \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=piper-pi05 \
  --inference.mode=chunk \
  --inference.semantics=piper-radians-gripper-metres-v1 \
  --inference.chunk_merge=aligned \
  --inference.refill_seconds=0.67 \
  --robot.type=piper_follower \
  --robot.id=right \
  --task="从桌子边缘的盒子里随机抓取方块，放到中间盒子上；再从中间盒子里随机抓取方块，放到边缘盒子上" \
  --fps=30

# Pi0.5 插插头模型 / 040000 / 30 Hz。
lerobot-rollout \
  --strategy.type=base \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=piper-pi05-plug \
  --inference.mode=chunk \
  --inference.semantics=piper-radians-gripper-metres-v1 \
  --inference.chunk_merge=aligned \
  --inference.refill_seconds=0.84 \
  --robot.type=piper_follower \
  --robot.id=right \
  --task="帮插头放入插座中" \
  --fps=30


############################ Pika + Piper DAgger 人机纠偏采集 ############################
# 前置：终端 1 保持运行 `bash examples/piper/start_pika_teleop_bridge.bash`
# 按键约定：
#   Space : 暂停策略 / 恢复策略自主运行
#   Tab   : （暂停态下）开始人工纠偏录帧（自动以机械臂当前姿态零跳变锚定 Pika）/ 再次按 Tab 结束并保存该条纠偏 episode
#   Pika手柄按键 : 纠偏过程中随时松开/重锚空间离合（移动手部位置而机械臂不动）
#   ESC   : 结束采集并保存数据集
# 注意：dataset.repo_id 仓库名必须以 rollout_ 开头；dataset.fps 必须与 --fps 相等。
# 如需同时录制自主段与纠偏段，追加 --strategy.record_autonomous=true。

# 1) 本地 Pi0.5 + RTC + Pika DAgger（双相机 / 10 Hz）
lerobot-rollout \
  --strategy.type=dagger \
  --strategy.num_episodes=30 \
  --inference.type=rtc \
  --inference.rtc.enabled=true \
  --inference.rtc.mode=guided \
  --inference.rtc.execution_horizon=10 \
  --inference.rtc.max_guidance_weight=10 \
  --inference.queue_threshold=10 \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --teleop.type=pika_leader \
  --teleop.id=pika \
  --policy.path=outputs/models/lazycc/pi05_pick_and_place_piper \
  --policy.compile_model=false \
  --dataset.repo_id=${HF_USER}/rollout_piper_dagger_pi05 \
  --dataset.no_stamp=true \
  --dataset.root=outputs/datasets/rollout_piper_dagger_pi05 \
  --dataset.single_task="pick up the object and place it in the container" \
  --dataset.fps=10 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --task="pick up the object and place it in the container" \
  --fps=10 \
  --device=cuda

# 2) 本地 ACT + Pika DAgger（双相机 / 10 Hz）
lerobot-rollout \
  --strategy.type=dagger \
  --strategy.num_episodes=30 \
  --inference.type=sync \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --robot.cameras="$PIPER_MODEL_CAMERAS" \
  --teleop.type=pika_leader \
  --teleop.id=pika \
  --policy.path=outputs/models/act_pick_and_place/checkpoints/006000/pretrained_model \
  --policy.n_action_steps=30 \
  --dataset.repo_id=${HF_USER}/rollout_piper_dagger_act \
  --dataset.no_stamp=true \
  --dataset.root=outputs/datasets/rollout_piper_dagger_act \
  --dataset.single_task="Pick up the object" \
  --dataset.fps=10 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --task="Pick up the object" \
  --fps=10 \
  --device=cuda

# 3) 远程推理 Pi0.5 (30 Hz 三相机) + 本地 Pika DAgger
lerobot-rollout \
  --strategy.type=dagger \
  --strategy.num_episodes=30 \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=piper-pi05 \
  --inference.mode=chunk \
  --inference.semantics=piper-radians-gripper-metres-v1 \
  --inference.chunk_merge=aligned \
  --inference.refill_seconds=0.67 \
  --robot.type=piper_follower \
  --robot.id=right \
  --robot.motors.can_name=can0 \
  --teleop.type=pika_leader \
  --teleop.id=pika \
  --dataset.repo_id=${HF_USER}/rollout_piper_dagger_remote \
  --dataset.no_stamp=true \
  --dataset.root=outputs/datasets/rollout_piper_dagger_remote \
  --dataset.single_task="从桌子边缘的盒子里随机抓取方块，放到中间盒子上；再从中间盒子里随机抓取方块，放到边缘盒子上" \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --task="从桌子边缘的盒子里随机抓取方块，放到中间盒子上；再从中间盒子里随机抓取方块，放到边缘盒子上" \
  --fps=30

# 4) DAgger 数据后处理（移除 intervention 列并与原数据集 merge 后微调）
lerobot-edit-dataset \
  --repo_id=${HF_USER}/rollout_piper_dagger_pi05 \
  --root=outputs/datasets/rollout_piper_dagger_pi05 \
  --new_repo_id=${HF_USER}/piper_dagger_clean \
  --new_root=outputs/datasets/piper_dagger_clean \
  --operation.type=remove_feature \
  --operation.feature_names="['intervention']" \
  --push_to_hub=false

lerobot-edit-dataset \
  --new_repo_id=${HF_USER}/piper_demo_plus_dagger \
  --new_root=outputs/datasets/piper_demo_plus_dagger \
  --operation.type=merge \
  --operation.repo_ids="['${HF_USER}/piper_pick_and_place', '${HF_USER}/piper_dagger_clean']" \
  --operation.roots="['outputs/datasets/piper_pick_and_place', 'outputs/datasets/piper_dagger_clean']" \
  --push_to_hub=false
