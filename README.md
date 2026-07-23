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

部署电脑已经从课题组环境克隆出 `rdp_deploy` Conda 环境。激活后只补充项目普通依赖：

```bash
conda activate rdp_deploy
bash setup_conda.sh
```

克隆环境已经包含并验证了以下硬件依赖，不要重新安装或升级：

- `flexivrdk==1.9.0`
- `r3kit==0.0.2`
- `xensegripper==1.3.0`
- `xensesdk==2.0.0`
- `pyrealsense2==2.53.1.4623`

当前克隆环境使用 Python 3.10，而 Ubuntu 24.04 的 ROS2 Jazzy apt 包使用 Python 3.12。机械臂和硬件 SDK 检查可以在该环境中运行，但 `rclpy` 发布与采集节点只有在 Python ABI 兼容后才能启动。不要用 pip 安装另一个 `rclpy` 来绕过这个问题。

ROS2 Python 包由系统 Jazzy 提供，不写入 `requirements.txt`。检查时执行：

```bash
sudo apt install ros-jazzy-rclpy ros-jazzy-message-filters \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs

conda activate rdp_deploy
source /opt/ros/jazzy/setup.bash
python scripts/check_imports.py --scope ros
```

## 2. 配置

主要配置文件：

```bash
configs/deploy_wipedish_sensor_only.yaml
```

重点检查这些字段：

- `robot.robot_id`：Rizon 机械臂 ID，默认 `Rizon4s-063586`。
- `robot.tool_name`：Rizon 工具名，默认 `hapticexoteleop`。
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
python scripts/check_imports.py --scope core
python scripts/check_imports.py --scope hardware
python scripts/check_robot_connection.py --config configs/deploy_wipedish_sensor_only.yaml
python scripts/check_publisher_config.py --config configs/deploy_wipedish_sensor_only.yaml
```

`check_robot_connection.py` 不依赖 ROS2，可直接验证 Rizon 和夹爪。完整发布前，必须保证 ROS 检查通过：

```bash
source /opt/ros/jazzy/setup.bash
python scripts/check_imports.py --scope ros
python scripts/check_ros_topics.py --config configs/deploy_wipedish_sensor_only.yaml
```

## 4. 运行

只启动发布节点：

```bash
conda activate rdp_deploy
source /opt/ros/jazzy/setup.bash
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
rdp_deploy/robots/rizon.py 中的 Rizon(tool_name="hapticexoteleop")
```

不会调用 `/move_tcp`、`/move_gripper` 等 HTTP 运动接口，也不会在 `RDP_deploy` 内实现动作下发。
