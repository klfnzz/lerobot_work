# Piper 使用说明

当前代码为官方 LeRobot 0.7.0，Piper 驱动已内置。
项目目录与 Conda 环境统一使用不带版本的 **`lerobot_piper`**，后续升级沿用此名称。

## 环境与命令笔记

```bash
source /home/qwe/lerobot/activate_lerobot.bash
cd /home/qwe/lerobot/lerobot_piper
python -c 'import lerobot; print(lerobot.__version__, lerobot.__file__)'
```

也可用 `conda activate lerobot_piper` 选择环境；上面的激活脚本还会设置所需动态库与代理格式。

[command.sh](../command.sh) 保留原来的分段命令笔记形式。先执行“环境”段落，再按任务复制对应段落的命令到终端执行。
需要改模型、数据集、CAN 或运行参数时，直接编辑命令中的相应参数。

| 任务 | command.sh 中的段落 | 当前入口 |
| --- | --- | --- |
| ACT / Diffusion 训练 | ACT 训练、Diffusion / PushT | `lerobot-train` |
| 恢复训练 | 从 checkpoint 续训 | `lerobot-train --config_path=... --resume=true` |
| 采集、回放 | 遥操作与采集 | `lerobot-teleoperate`、`lerobot-record`、`lerobot-replay` |
| 本地模型推理 | ACT 本地推理、Pi0.5 本地推理 | `lerobot-rollout` |
| 推理并保存评估 episode | ACT 003000 / 006000 评估段落 | `lerobot-rollout --strategy.type=episodic` |
| 远程推理 | GPU 服务端、本地 Piper 客户端 | `lerobot-policy-server`、`lerobot-rollout --inference.type=remote` |

本机保留 Python 3.12.14、PyTorch 2.11.0+cu130、torchvision 0.26.0+cu130，
pyAgxArm 固定在 `57a95a8`。重新安装本地依赖：

```bash
uv pip install --python "$CONDA_PREFIX/bin/python" \
  -e '.[piper,core_scripts,remote,training,pi,diffusion,pusht]' 'httpx[socks]'
uv pip check --python "$CONDA_PREFIX/bin/python"
```

视频编码和 TorchCodec 解码使用本机 Conda FFmpeg 7.1.1 的共享库。
激活脚本将桌面代理导出的 `socks://` 别名规范为 `socks5h://`，保留原代理端点。
命令笔记使用 `HF_ENDPOINT=https://hf-mirror.com`，可按需要改为官方 Hub 地址。
本地 ACT 推理及不上传的本地数据训练无需 Hub 登录；上传时运行 `hf auth login`。
Pi0.5 所需的 PaliGemma tokenizer 本机已缓存；新机器需准备其仓库访问条件。

新机器也可用 Python >=3.12 和项目锁文件安装：

```bash
uv sync --locked --extra piper --extra core_scripts --extra remote \
  --extra training --extra pi --extra diffusion --extra pusht
source .venv/bin/activate
```

FFmpeg 安装方式见 [安装文档](source/installation.mdx)。

## CAN、相机与动作约定

| 项目 | 当前配置 |
| --- | --- |
| Follower / Leader CAN | `can0` / `can_master1` |
| CAN 接口 / 速率 | `socketcan` / 1,000,000 |
| `pikaGripperDepthCamera` | `230322272075` |
| `highDepthCamera` | `233622078698` |
| `sideDepthCamera` | `109122072082` |
| 相机图像 | RGB，640×480，30 FPS |
| 动作顺序 | `joint_1.pos` … `joint_6.pos`，`gripper.pos` |
| 单位 | 关节弧度、夹爪米 |
| 初始与驱动停机姿态 | `[0, 0, 0, 0, 0.52, 0, 0]` |

采集和 30 Hz 远程命令使用当前三相机配置。
现有本地 ACT、下载的 Pi0.5 和 10 Hz 远程模型使用训练时的双相机：
`pikaGripperDepthCamera=230322272075`、`sideDepthCamera=233622078698`。
命令笔记中的 `PIPER_MODEL_CAMERAS` 保存这套双相机配置，并通过 `--robot.cameras` 显式传入。

相机名称、视角与服务端 YAML 特征需要匹配所选 checkpoint。
远端 30 Hz checkpoint 未保存在本机，使用前需按其训练配置选择相机视角。
名称中的 `DepthCamera` 沿用原命名，当前保存的是 RGB 图像。

Piper 声明了位置保持能力，异步等待时重发最后实际发送的完整位置目标。
夹爪负值截为零，非有限目标在发送前报错。
Follower 断开时先发送驱动停机姿态、等待 5 秒，再失能并释放相机。
`lerobot-rollout` 的 `return_to_initial_position` 是独立设置。
通用 `lerobot-find-joint-limits`、`lerobot-setup-motors` 不适用于此 Piper 驱动。

## 数据与训练

采集示例写入 `outputs/datasets/piper_pick_and_place`，ACT 训练示例读取同一路径。
已有数据时修改 `--dataset.repo_id` 和 `--dataset.root`；从 Hub 训练时可省略 root。
采集默认不上传，回放使用相同的 repo_id、root 和指定 episode。

训练输出分别为 `outputs/train/act_pick_and_place` 和 `outputs/train/dp_pusht`。
ACT 显式使用 `batch_size=64, chunk_size=30, n_action_steps=30`；
Diffusion 使用 `batch_size=64, clip_sample=false`，可按显存调整 batch_size。
训练日志使用 `--trackio.enable=true`，默认记录在本地。
续训的 `config_path` 指向实际 checkpoint 下的 `pretrained_model/train_config.json`。

## 本地模型

ACT 直接使用 `outputs/models/act_pick_and_place/checkpoints/` 下的原 checkpoint。
命令笔记提供 006000 的同步推理，以及 003000、006000 的分 episode 评估命令。
003000 使用逐步时序集成，006000 每次执行 30 个动作。
已有同名评估目录时，换用新 `dataset.root`，或使用 `--resume=true` 继续采集。

Pi0.5 的兼容目录使用永久名称 `outputs/models/lazycc/pi05_pick_and_place_piper`：

```bash
python examples/piper/prepare_checkpoint.py \
  --source=outputs/models/lazycc/pi05_pick_and_place \
  --destination=outputs/models/lazycc/pi05_pick_and_place_piper
```

工具将已知 Pika 动作名称按原顺序映射到 Piper 的七个分量，保存调整后的配置，
权重和 processor 引用原文件。原始模型目录需保留。
工具拒绝未知动作顺序，以及需单独迁移的相对动作配置。

本地 Pi0.5 命令使用该兼容目录，提供同步与 guided RTC 两种模式，默认关闭编译。
RTC 参数为 execution_horizon=10、max_guidance_weight=10、queue_threshold=10。
trained RTC 需要 checkpoint 已在训练时启用相应能力。

## 远程推理

GPU 服务端与 Piper 客户端使用相同版本代码和 `[remote]` 依赖，模型放在服务端。
只有连接 Piper 硬件的机器需要 `[piper]`，Pi0.5 服务端还需 `[pi]`。
服务端先准备兼容模型目录，再启动 `lerobot-policy-server`；客户端运行
`lerobot-rollout --inference.type=remote`。

| 模型 | deployment | 模板 | 相机 / FPS |
| --- | --- | --- | --- |
| Pi0.5 020000 | `piper-pi05-10hz` | `examples/piper/remote_pi05_10hz.yaml` | 双相机 / 10 |
| Pi0.5 030000 | `piper-pi05` | `examples/piper/remote_pi05.yaml` | 三相机 / 30 |
| Pi0.5 插插头 040000 | `piper-pi05-plug` | `examples/piper/remote_pi05.yaml`，显式覆盖模型路径 | 三相机 / 30 |

笔记保留原远端模型路径中的 `pi05_pick_place`、
`muti_pick_and_place_task_single_arm_lerobot_30hz`、`plug_and_unplug_task_raw_30hz`。
远端进入实际源码目录并激活其 Python 环境后，选择一组服务端命令执行。
服务端 `--model.compile_model=false` 显式关闭编译；省略则使用 checkpoint 设置。

服务端默认监听 `tcp/127.0.0.1:7447`。本地单独终端保持隧道：

```bash
ssh -N -L 7447:127.0.0.1:7447 manager@172.29.220.198
```

另一个本地终端执行命令笔记中的环境设置，再复制匹配的客户端命令。
客户端与服务端的 deployment、FPS、相机特征和动作约定必须一致。
三组客户端的 refill_seconds 分别为 1.0、0.67、0.84，使用 aligned 动作队列合并。
可按实际延迟调整补充阈值；需要混合重叠目标时，用 `inference.blend_*` 参数显式选择关节。

## 检查与备份

```bash
python ../migration-support/verify_environment.py --profile lerobot --cuda
uv run --active --no-project pytest -q tests/robots/test_piper.py tests/robots/test_piper_checkpoint.py
```

改名前的文件与环境清单位置见 `../.upgrade-backups/LATEST_RENAME`。
首次升级前的源码、命令和未提交改动位置见 `../.upgrade-backups/LATEST`。
命名更新包含模型链接、源码 editable 安装与激活脚本；本次未执行机械臂运动。
