# RDP Deploy

面向真机的独立传感器采集包，直接使用硬件 SDK。

当前功能：

- 读取 Flexiv Rizon 状态和 Xense 夹爪宽度。
- 读取 RealSense 彩色图像。
- 读取 Xense 图像、Marker 偏移和合力。
- 按时间戳同步为统一 observation。
- 保存单帧 snapshot 或连续 stream。
- 不加载模型，不发送机械臂运动指令。

## 1. 环境

默认已经进入本目录：

```bash
cd RDP_deploy
conda activate rdp_deploy
bash setup_conda.sh
```

脚本只补装项目缺少的普通依赖。以下硬件包沿用克隆环境中的已验证版本，不会被 `requirements.txt` 安装或升级：

- `flexivrdk==1.9.0`
- `r3kit==0.0.2`
- `xensegripper==1.3.0`
- `xensesdk==2.0.0`
- `pyrealsense2==2.53.1.4623`

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
- `devices.realsense.cameras`：RealSense 序列号、分辨率和帧率。
- `devices.xense.sensors`：Xense 序列号、MAC、GPU 和输出项。
- `runtime.expected_fps`：observation 生成频率。
- `runtime.sync_slop`：各设备样本允许的最大时间差。
- `observation.required_keys`：采集结果必须包含的字段。

## 3. 检查

先验证依赖和纯数据链路：

```bash
python scripts/check_imports.py
python scripts/check_direct_pipeline.py
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

## 5. 安全

采集期间只读取状态和传感器数据。Rizon 初始化会连接机械臂、使能、切换工具并执行力传感器清零，但不会发送位姿、关节或夹爪运动命令。
