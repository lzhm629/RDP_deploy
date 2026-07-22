# RDP Deploy

第一阶段只做传感器采集和通信检查，不加载模型、不发送机械臂运动指令。

## 环境

本目录只提供 `requirements.txt` 和可选初始化脚本，不会自动创建环境。

```bash
python3.10 -m venv rdp_deploy_venv
source rdp_deploy_venv/bin/activate
pip install --upgrade pip
pip install -r RDP_deploy/requirements.txt
```

每次运行前先加载 ROS2 和项目路径：

```bash
source /opt/ros/humble/setup.bash
source rdp_deploy_venv/bin/activate
export PYTHONPATH=$PWD/reactive_diffusion_policy:$PWD/RDP_deploy:$PYTHONPATH
```

## 验证顺序

先在真机电脑分别启动已有服务：

```bash
python reactive_diffusion_policy/teleop.py task=real_wipe_two_realsense_one_gelsight_one_mctac_24fps
python reactive_diffusion_policy/camera_node_launcher.py task=real_wipe_two_realsense_one_gelsight_one_mctac_24fps
```

然后逐步检查：

```bash
python RDP_deploy/scripts/check_imports.py
python RDP_deploy/scripts/check_robot_server.py --config RDP_deploy/configs/deploy_wipedish_sensor_only.yaml
python RDP_deploy/scripts/check_device_mapping.py --config RDP_deploy/configs/deploy_wipedish_sensor_only.yaml
python RDP_deploy/scripts/check_ros_topics.py --config RDP_deploy/configs/deploy_wipedish_sensor_only.yaml
python RDP_deploy/scripts/collect_sensor_snapshot.py --config RDP_deploy/configs/deploy_wipedish_sensor_only.yaml
python RDP_deploy/scripts/collect_sensor_stream.py --config RDP_deploy/configs/deploy_wipedish_sensor_only.yaml --duration 10
```

## 当前边界

- 只读 robot server：只调用 `/get_current_robot_states` 和 `/get_current_tcp/{side}`。
- 不实现 `/move_tcp`、`/move_gripper` 等运动接口。
- 采集结果默认保存到 `RDP_deploy/logs`。
- ROS2 Python 包来自系统 ROS2 Humble，不通过 pip 安装。
