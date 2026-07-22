# RDP Deploy

面向 Ubuntu 24.04 + ROS2 Jazzy 的独立传感器采集部署包。

当前范围：

- 启动机器人状态、RealSense、Xense 的 ROS2 发布节点。
- 从 ROS2 topics 同步采集 observation。
- 保存单帧 snapshot 或连续 stream 数据，方便上机前验证。
- 不加载模型，不发送任何机械臂运动指令。
- 机械臂连接逻辑已按 Forcemimic 的 Rizon 配置方式迁移到本目录内。

## 1. 环境

默认你已经进入本目录：

```bash
cd RDP_deploy
```

创建并激活 venv：

```bash
python3.12 -m venv rdp_deploy_venv
source rdp_deploy_venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

每次运行前加载 ROS2 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
source rdp_deploy_venv/bin/activate
export PYTHONPATH=$PWD:$PYTHONPATH
```

ROS2 Python 包通过 apt 安装，不通过 pip 安装：

```bash
sudo apt install ros-jazzy-message-filters
```

如果启用硬件发布节点，还需要在部署电脑安装硬件 SDK：

- RealSense：系统 librealsense 驱动，以及 Python 包 `pyrealsense2`。
- Xense 传感器：`xensesdk`。
- Rizon 机械臂：Flexiv RDK Python 包 `flexivrdk`。
- Xense 夹爪：`r3kit`，或把 r3kit 的 Python 包根目录加入 `PYTHONPATH`。

## 2. 配置

主要配置文件：

```bash
configs/deploy_wipedish_sensor_only.yaml
```

重点检查这些字段：

- `robot.robot_id`：Rizon 机械臂 ID，默认 `Rizon4s-063586`。
- `robot.tool_name`：Rizon 工具名，默认 `xense_force`。
- `robot.gripper_id`、`robot.gripper_name`：Xense 夹爪配置。
- `publishers.realsense.cameras`：RealSense 序列号和发布 topic 名称。
- `publishers.xense.sensors`：Xense 序列号和发布 topic 名称。
- `sensors.subscribe_topics`：同步采集器订阅的 topic 列表。

默认配置会发布并采集：

- `/D405/color/image_raw`
- `/left_gripper_camera_1/color/image_raw`
- `/left_gripper_camera_1/marker_offset/information`
- `/left_gripper_camera_1/force_resultant`
- `/left_tcp_pose`
- `/left_gripper_state`
- `/left_tcp_vel`
- `/left_tcp_wrench`

## 3. 检查

按顺序执行：

```bash
python scripts/check_imports.py
python scripts/check_robot_connection.py --config configs/deploy_wipedish_sensor_only.yaml
python scripts/check_publisher_config.py --config configs/deploy_wipedish_sensor_only.yaml
```

如果发布节点已经启动，可以检查 ROS2 topics：

```bash
python scripts/check_ros_topics.py --config configs/deploy_wipedish_sensor_only.yaml
```

## 4. 运行

只启动发布节点：

```bash
python scripts/launch_publishers.py --config configs/deploy_wipedish_sensor_only.yaml
```

另开一个终端，采集单帧 snapshot：

```bash
python scripts/collect_sensor_snapshot.py --config configs/deploy_wipedish_sensor_only.yaml
```

采集 10 秒 stream：

```bash
python scripts/collect_sensor_stream.py --config configs/deploy_wipedish_sensor_only.yaml --duration 10
```

也可以一键启动发布节点并采集 10 秒：

```bash
python scripts/start_publishers_and_collect.py --config configs/deploy_wipedish_sensor_only.yaml --duration 10
```

采集结果保存在：

```bash
logs/
```

## 5. 安全边界

当前采集链路只读机器人状态，不发送动作指令。

机器人状态发布使用本目录内的：

```text
rdp_deploy/robots/rizon.py 中的 Rizon(tool_name="xense_force")
```

不会调用 `/move_tcp`、`/move_gripper` 等 HTTP 运动接口，也不会在 `RDP_deploy` 内实现动作下发。
