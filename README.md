# RDP Deploy

面向真机的独立采集与部署验证包，直接使用硬件 SDK。

当前功能：

- 读取 Flexiv Rizon 状态和 Xense 夹爪宽度。
- 读取 RealSense 腕部视觉图像。
- 按时间戳同步为统一 observation。
- 缓存连续 2 帧并生成模型输入格式。
- 保存单帧 snapshot 或连续 stream。
- 加载 RDP checkpoint，离线生成并解码动作。
- 支持实时 `shadow`、`hold` 和 `execute` 三种部署模式。
- 夹爪控制不进入运行链路，海绵由人工保持夹紧。

## 1. 环境

默认已经进入本目录：

```bash
cd RDP_deploy
conda activate rdp_deploy
bash setup_conda.sh
```

脚本只补装项目缺少的普通依赖。以下硬件包沿用克隆环境中的已验证版本，不会被 `requirements.txt` 安装或升级：

- `flexivrdk==1.9.0`
- `xensegripper==1.3.0`
- `pyrealsense2==2.53.1.4623`

`torch` 和 `torchvision` 建议继续使用 Conda 环境中与 CUDA 匹配的版本。模型推理还需要 `hydra-core`、`dill`、`einops`、`diffusers`、`zarr`、`timm` 和 `peft`。

## 2. 配置

主配置文件：

```text
configs/deploy_wipedish_sensor_only.yaml
```

重点字段：

- `robot.robot_id`：Rizon ID。
- `robot.tool_name`：Flexiv 控制器中的工具名，当前为 `hapticexoteleop`。
- `robot.gripper_id`：Xense 夹爪 ID。
- `devices.robot`：机械臂状态读取开关和频率。
- `devices.realsense.cameras`：腕部 RealSense 序列号、分辨率和帧率。
- `runtime.expected_fps`：observation 生成频率。
- `runtime.sync_slop`：各设备样本允许的最大时间差。
- `observation.history_size`：模型输入的连续帧数，当前为 2。
- `observation.required_keys`：采集结果必须包含的字段。

每个 observation 只包含：

- `left_wrist_img`：`(1, 2, 3, 240, 320)`，RGB，`float32`，范围 `[0, 1]`。
- `left_robot_tcp_pose`：`(1, 2, 9)`。
- `left_robot_gripper_width`：`(1, 2, 1)`。
- `left_robot_tcp_wrench`：`(1, 2, 6)`。

## 3. 检查

先验证依赖和纯数据链路：

```bash
python scripts/check_imports.py
python scripts/check_direct_pipeline.py
python scripts/check_offline_control.py
python scripts/check_control_pipeline.py
python scripts/check_deployment_runtime.py
```

检查硬件包、RealSense 序列号和配置：

```bash
python scripts/check_hardware_config.py \
  --config configs/deploy_wipedish_sensor_only.yaml
```

单独检查 Rizon、工具和夹爪连接：

```bash
python scripts/check_robot_connection.py \
  --config configs/deploy_wipedish_sensor_only.yaml
```

## 4. 采集

采集并保存单帧：

```bash
python scripts/collect_sensor_snapshot.py \
  --config configs/deploy_wipedish_sensor_only.yaml
```

连续采集 10 秒：

```bash
python scripts/collect_sensor_stream.py \
  --config configs/deploy_wipedish_sensor_only.yaml \
  --duration 10
```

采集结果保存在 `logs/`。

## 5. 离线控制验证

需要把两个 checkpoint 放到部署机：

- LDP checkpoint：`train_latent_diffusion_unet.../checkpoints/latest.ckpt`
- AT checkpoint：`train_vae.../checkpoints/latest.ckpt`

使用已经保存的 snapshot 验证完整推理和动作解码：

```bash
python scripts/validate_offline_control.py \
  --config configs/deploy_wipedish_sensor_only.yaml \
  --snapshot logs/snapshot_YYYYMMDD_HHMMSS.pkl
```

默认配置按 `RDP_deploy` 与 `wipedish_7_17` 同级存放：

```text
../wipedish_7_17/LDP/checkpoints/epoch=0340-train_loss=0.085.ckpt
../wipedish_7_17/AT/checkpoints/latest.ckpt
```

如果模型目录改变，可使用 `--checkpoint` 和 `--at-checkpoint` 临时覆盖。

输出包括：

- 潜动作、相对动作和绝对动作的 shape。
- 前几步 10 维解码动作。
- 转换后的 Flexiv 目标位姿 `[x, y, z, qw, qx, qy, qz]`。
- `commands_sent: 0` 安全状态。

主 checkpoint 不包含独立 AT 的完整权重，因此两个 checkpoint 都必须提供。夹爪动作只显示、不执行，任务开始前由人工保持海绵夹紧。

## 6. 安全

采集期间只读取状态和传感器数据。Rizon 初始化会连接机械臂、使能、切换工具并执行力传感器清零，但不会发送位姿、关节或夹爪运动命令。

`validate_offline_control.py` 只读取 snapshot 和 checkpoint，不导入硬件控制类、不连接机械臂，也没有命令下发接口。

## 7. 完整部署

完整部署会先加载模型，再初始化硬件。启动脚本会自动优先使用 PyTorch 自带的 cuDNN，同时保留 Flexiv 所需的动态库路径。

### 7.1 Shadow

实时采集、推理、解码和安全检查，但不调用机械臂运动接口：

```bash
python scripts/run_deployment.py \
  --config configs/deploy_wipedish_sensor_only.yaml \
  --mode shadow \
  --duration 10
```

`shadow` 是默认模式。它会连接并使能 Rizon、切换工具并执行力传感器清零，但发送的运动命令数始终为 0。

### 7.2 Hold

以 90 Hz 发送启动时读取到的当前 TCP 位姿，用于单独验证笛卡尔控制模式、通信和停止流程：

```bash
python scripts/run_deployment.py \
  --config configs/deploy_wipedish_sensor_only.yaml \
  --mode hold \
  --duration 3 \
  --confirm-motion MOVE_RIZON4S_063586
```

第一次运行 `hold` 时应保持急停可触达，并确认机械臂没有明显位移。任何异常都会停止发送并切换到 `IDLE`。

### 7.3 Execute

只有 `shadow` 和 `hold` 都通过后才运行：

```bash
python scripts/run_deployment.py \
  --config configs/deploy_wipedish_sensor_only.yaml \
  --mode execute \
  --duration 3 \
  --confirm-motion MOVE_RIZON4S_063586
```

首次执行建议保持 `3` 秒。确认轨迹、外力和工作空间日志正常后，再逐步增加持续时间。

### 7.4 控制参数

`deployment` 配置控制运行频率和看门狗：

- 模型控制频率为 24 Hz。
- Latent Diffusion 推理频率为 6 Hz。
- 潜动作轨迹每 16 个控制周期更新。
- 丢弃潜动作前 4 帧进行延迟补偿。
- Rizon 目标以 90 Hz 发送。
- observation 或目标超过 0.25 秒未刷新即停止。

`safety` 同时限制：

- 固定笛卡尔工作空间。
- 相对启动位姿的最大移动范围。
- 外力、外力矩、目标跳变和 TCP 跟踪误差。
- 最大线速度 `0.02 m/s` 和最大角速度 `0.05 rad/s`。
- checkpoint 归一化后的低维观测绝对值，默认上限为 `20`。

安全限制超出时不会裁剪到工作空间边界继续运行，而是立即终止并进入 `IDLE`。速度限制会逐步限速。

### 7.5 日志

每次运行生成：

```text
logs/deployment_YYYYMMDD_HHMMSS.jsonl
```

日志包含原始候选目标、安全目标、当前 TCP、wrench、限速状态、推理耗时、计划代数和停止原因。夹爪模型输出不会发送。
