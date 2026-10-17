# Piper / LeRobot 0.6.1

统一工作目录：`/home/qwe/lerobot_exp/lerobot_piper061`。Piper 已直接集成到
`src/lerobot/motors/piper`、`src/lerobot/robots/piper_follower` 和
`src/lerobot/teleoperators/piper_leader`，不再需要独立 Piper 插件。

## 环境

本机使用已有 Python 3.12 Conda 环境：

```bash
conda activate lerobot_piper061
cd /home/qwe/lerobot_exp/lerobot_piper061
python -c 'import lerobot; print(lerobot.__version__, lerobot.__file__)'
```

新机器在本目录安装（Python >=3.12）：

```bash
python -m pip install -e '.[piper,core_scripts,async,training]'
```

需要 Pi0/Pi0.5 时再加 `pi` extra，Diffusion 加 `diffusion`。原来装过独立插件的环境，
先运行 `python -m pip uninstall lerobot_robot_piper`，避免重复注册。
旧 `lerobot_piper` Python 3.10 环境不支持 0.6.1；本机已解除其中旧 LeRobot 的 editable 安装。

## 保留的硬件配置

| 项目 | 默认值 |
| --- | --- |
| Follower / Leader CAN | `can0` / `can_master1` |
| CAN 接口 / 速率 | `socketcan` / 1,000,000 |
| 手部相机 | `pikaGripperDepthCamera`: `230322272075` |
| 侧面相机 | `sideDepthCamera`: `233622078698` |
| 两台相机图像 | 640×480，30 FPS |
| 动作顺序 | `joint_1.pos` … `joint_6.pos`，`gripper.pos` |
| 单位 | 关节弧度，夹爪米 |
| 夹爪力 / CAN 检查 | 1.0 / 启用 |
| 初始与停机姿态 | `[0, 0, 0, 0, 0.52, 0, 0]` |

Follower 停机先发送安全姿态、等待 5 秒，再失能并释放相机。
配置类和相机名称保持兼容已训练模型；图像特征仍由 LeRobot 添加一次 `observation.images.` 前缀。
可通过 `--robot.motors.can_name=can1` 或 `--teleop.motors.can_name=can_master1` 覆盖接口。
通用 `lerobot-find-joint-limits` 使用角度制运动学，`lerobot-setup-motors` 使用电机 ID 设置接口，
两者不适用于当前 Piper 实现；未将 Piper 注册到这两个命令。
相机可通过 `--robot.cameras='{...}'` 显式替换；空字典沿用默认双相机行为。
校准目录沿用 061 插件的标准缓存位置；需要旧版相对目录时，显式传入
`--teleop.calibration_dir=.cache/calibration/piper_leader`。

遥操作（会启用机械臂）：

```bash
lerobot-teleoperate --robot.type=piper_follower --robot.id=right \
  --teleop.type=piper_leader --teleop.id=leader --fps=30
```

## 本地模型与推理

原有 `outputs/` 已整体迁入本目录，包括 ACT 的 003000/006000/009000 checkpoint、
`models/lazycc/pi05_pick_and_place`、评估数据和相机图片。
0.6.1 本地策略推理使用 `lerobot-rollout`：

```bash
lerobot-rollout \
  --strategy.type=base \
  --policy.path=outputs/models/act_pick_and_place/checkpoints/006000/pretrained_model \
  --robot.type=piper_follower --robot.id=right \
  --task="Pick up the object" --fps=10 --device=cuda
```

异步服务端使用同一份 0.6.1 源码及对应策略依赖：

```bash
python -m lerobot.async_inference.policy_server \
  --host=127.0.0.1 --port=8080 --fps=30 --inference_latency=0.1 --obs_queue_timeout=2
```

本地建立原来的 SSH 隧道后，再运行客户端（远端模型路径保持原值）：

```bash
ssh -N -L 8080:127.0.0.1:8080 manager@172.29.220.198
```

```bash
python -m lerobot.async_inference.robot_client \
  --server_address=127.0.0.1:8080 --robot.type=piper_follower \
  --policy_type=pi05 \
  --pretrained_name_or_path=/home/manager/wd/pi_med/lerobot/outputs/train/muti_pick_and_place_task_single_arm_lerobot_30hz/checkpoints/030000/pretrained_model \
  --policy_device=cuda --client_device=cpu \
  --task="从桌子边缘的盒子里随机抓取方块，放到中间盒子上；再从中间盒子里随机抓取方块，放到边缘盒子上" \
  --fps=30 --actions_per_chunk=50 --chunk_size_threshold=0.4 --aggregate_fn_name=weighted_average
```

旧 command.sh 的另一组设置为 fps=10、chunk_size_threshold=0.2，远端模型为
`/home/manager/wd/pi_med/lerobot/outputs/train/pi05_pick_place/checkpoints/020000/pretrained_model`，
任务为 `pick up the object and place it in the container`。原始完整笔记保存在归档中。

## 训练偏好

LeRobot 核心与默认参数以 0.6.1 为准。原来对旧源码的全局修改改为显式参数：

- ACT：在训练命令追加 `--batch_size=64 --policy.chunk_size=30 --policy.n_action_steps=30`。
- Diffusion：追加 `--batch_size=64 --policy.clip_sample=false`。

旧笔记中的 `demo_data_v3_video` 本来就不存在；训练时应指定真实的数据集路径。

## 旧文件恢复

`consolidation/archives/` 保留旧源码、Git 历史、未提交修改、command.sh、任务记录和日志的
压缩包。模型和评估数据已放到 `outputs/`，未重复压缩。归档不参与 Python 导入或安装。
如需取回单个文件，使用 `tar -xzf <archive> -C <temporary-directory> <member>`，
不要将旧版 src 覆盖到当前 0.6.1。
