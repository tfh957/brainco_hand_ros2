# BrainCo Hand ROS2 使用说明

[English](README.md) | [简体中文](README_CN.md)

本文档用于快速说明 BrainCo Revo2 灵巧手 ROS2 包的使用方式，重点覆盖当前项目中的 **RM65 机械臂 + Revo2 右手真机系统**。

## 1. 项目用途

本仓库提供 Revo2 灵巧手在 ROS2 Humble 下的模型、驱动、仿真、MoveIt 配置，以及 RM65 机械臂末端安装 Revo2 的集成演示。

主要功能：

- Revo2 灵巧手 URDF 模型与 RViz 可视化
- Revo2 单手/双手 ros2_control 驱动
- Gazebo 仿真
- MoveIt 运动规划
- RM65 + Revo2 组合机器人模型与控制配置
- RM65 末端 485 控制 Revo2 的桥接节点 `rm_revo2_bridge`
- `tools/` 下的单点位姿、连续动作、手势快捷命令和 TCP 示例脚本

## 2. 当前真机系统的控制逻辑

当前硬件连接方式：

```text
电脑 --网线--> RM65 控制器 --末端 485/电源线--> Revo2 灵巧手
```

这意味着 Revo2 手不是直接插到电脑的 `/dev/ttyUSB*`，而是挂在 RM65 末端 485 上。因此，不能直接用原始 `brainco_hand_driver revo2_system.launch.py` 去扫描电脑串口控制这只手。

当前推荐控制链：

```text
机械臂状态/控制：
RM65 真机 <--> rm_ws/rm_driver <--> /joint_states, /rm_driver/movej_canfd_cmd

手部控制：
/revo2_hand_controller/joint_trajectory
        -> rm_revo2_bridge
        -> RM SDK
        -> RM65 末端 Modbus/485
        -> Revo2 灵巧手

MoveIt/RViz：
读取 /joint_states 显示机器人状态，并通过 /rm_group_controller、/revo2_hand_controller 发送轨迹
```

说明：

- `rm_ws` 负责连接 RM65 真机，发布机械臂关节状态。
- `brainco_ws` 负责启动 MoveIt、RViz 和 Revo2 末端 485 桥接。
- `rm_revo2_bridge` 默认使用 `sdk` 后端控制手，稳定性优于通过 `/rm_driver/write_modbus_rtu_registers_cmd` 转发。
- `/joint_states` 是标准 ROS 关节状态话题，机械臂和手的状态都可以出现在这里，供 `robot_state_publisher` 和 MoveIt 使用。

## 3. 目录结构

```text
brainco_hand_ros2/
├── revo2_description/                         # Revo2 URDF/模型
├── brainco_hardware/brainco_hand_driver/      # 原始 Revo2 硬件驱动
├── brainco_gazebo/                            # Gazebo 仿真
├── brainco_moveit_config/                     # Revo2 单独 MoveIt 配置
├── revo2_with_rm65_demo/
│   ├── gazebo_rm_65_6f_with_revo2_demo/       # RM65+Revo2 Gazebo 示例
│   ├── rm65_with_revo2_right_moveit_config/   # RM65+Revo2 MoveIt 配置
│   └── rm_revo2_bridge/                       # RM65 末端 485 到 Revo2 的桥接节点
└── tools/                                     # 调试和动作脚本
```

## 4. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- 已编译并可运行的 `rm_ws`
- 当前项目路径示例：`/home/fishros/rmrobot/brainco_ws`
- RM65 默认 IP：`192.168.1.18`

如果 RM65 IP 改过，需要同步修改 launch 参数或配置。

## 5. 获取代码和编译

如果是第一次搭建工作空间，可以按下面方式获取代码和依赖仓库：

```bash
mkdir -p ~/rmrobot/brainco_ws/src
cd ~/rmrobot/brainco_ws/src
git clone https://github.com/BrainCoTech/brainco_hand_ros2.git
cd brainco_hand_ros2
vcs import . < brainco_hand.repos --recursive --skip-existing
```

如果系统没有 `vcs`：

```bash
sudo apt-get install python3-vcstool
```

在 `brainco_ws` 根目录执行：

```bash
cd ~/rmrobot/brainco_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore stark_ethercat_interface stark_ethercat_driver brainco_hand_ethercat_driver
source install/setup.bash
```

如果只改了 `tools/` 下直接用 `python3` 运行的脚本，通常不需要重新 `colcon build`。

如果改了 `rm_revo2_bridge` 或 launch 文件，建议重新编译：

```bash
cd ~/rmrobot/brainco_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_revo2_bridge rm65_with_revo2_right_moveit_config
source install/setup.bash
```

## 6. RM65 + Revo2 真机启动流程

### 终端 1：启动 RM65 驱动

```bash
cd ~/rmrobot/rm_ws
source install/setup.bash
./start_rm65.sh
```

正常情况下可以看到 RM65 连接成功，并持续发布机械臂状态。

检查机械臂状态：

```bash
ros2 topic echo /joint_states --once
```

应能看到 `joint1` 到 `joint6`。

### 终端 2：启动 MoveIt、RViz 和 Revo2 桥接

```bash
cd ~/rmrobot/brainco_ws
source install/setup.bash
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py
```

当前 launch 默认值：

- `use_fake_hardware:=false`
- `use_rm_revo2_bridge:=true`
- `rm_robot_ip:=192.168.1.18`
- `hand_backend:=sdk`

所以一般不需要再手动追加这些参数。

如果需要指定 IP：

```bash
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py rm_robot_ip:=192.168.1.18
```

检查桥接节点：

```bash
ros2 node list | grep rm_revo2_bridge
ros2 param get /rm_revo2_bridge hand_backend
```

期望：

```text
/rm_revo2_bridge
String value is: sdk
```

## 7. 常用控制命令

### 7.1 控制 Revo2 张开

```bash
ros2 topic pub --once /revo2_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_thumb_metacarpal_joint','right_thumb_proximal_joint','right_index_proximal_joint','right_middle_proximal_joint','right_ring_proximal_joint','right_pinky_proximal_joint'], points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 1}}]}"
```

### 7.2 控制 Revo2 握拳

```bash
ros2 topic pub --once /revo2_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_thumb_metacarpal_joint','right_thumb_proximal_joint','right_index_proximal_joint','right_middle_proximal_joint','right_ring_proximal_joint','right_pinky_proximal_joint'], points: [{positions: [1.0,1.2,1.2,1.2,1.2,1.2], time_from_start: {sec: 1}}]}"
```

### 7.3 使用快捷函数

```bash
cd ~/rmrobot/brainco_ws/src/src/brainco_hand_ros2
source tools/revo2_cmds.sh
revo2_open
revo2_fist
revo2_pub 0.3 0.8 1.2 1.2 1.2 1.2 1
```

`revo2_pub` 最后一个参数是动作时间 `sec`，单位秒。

## 8. tools 脚本

进入源码目录：

```bash
cd ~/rmrobot/brainco_ws/src/src/brainco_hand_ros2
```

### 8.1 机械臂末端移动到单个位姿

```bash
python3 tools/move_arm_to_pose.py \
  --x -0.20 --y 0.00 --z 0.70 \
  --roll 0 --pitch 0 --yaw 0 \
  --sec 6.0
```

说明：

- `x y z` 是目标末端位置，单位米。
- `roll pitch yaw` 是欧拉角，单位弧度。
- `sec` 是发送给轨迹控制器的期望运动时间。

### 8.2 连续动作序列：机械臂 + 手

```bash
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json
```

常用调试参数：

```bash
# 只跑机械臂，不控制手
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json --skip-hand

# 手部用 topic 模式，并按 hand_sec 等待
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json --hand-mode topic --hand-topic-wait

# 增加 IK 计算时间
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json --ik-timeout 1.0 --ik-attempts 50
```

关于 IK warning：

```text
Collision-aware IK failed; retrying IK without collision checking.
```

这表示 MoveIt 的碰撞感知 IK 太严格，脚本改用不带碰撞检查的 IK 重试。脚本仍会保留关节跳变检查，避免突然跳到跨度过大的解。

更多工具说明见：[`tools/README.md`](tools/README.md)。

## 9. 原始 Revo2 单独驱动模式

如果 Revo2 直接通过 USB/串口连接电脑，而不是挂在 RM65 末端 485 上，可以使用原始驱动：

```bash
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right
```

原始右手控制话题通常是：

```text
/right_revo2_hand_controller/joint_trajectory
```

当前 RM65 末端 485 桥接模式使用的话题是：

```text
/revo2_hand_controller/joint_trajectory
```

二者不要混淆。

## 10. 仿真和纯 MoveIt 模式

### Revo2 单独 Gazebo 仿真

```bash
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=right
```

### Revo2 单独 MoveIt FakeSystem

```bash
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py
```

### RM65 + Revo2 Gazebo 仿真

```bash
ros2 launch gazebo_rm_65_6f_with_revo2_demo gazebo_rm_65_6f_with_revo2.launch.py
```

## 11. 状态检查和排错

### 11.1 检查节点

```bash
ros2 node list
```

关键节点：

```text
/rm_driver
/udp_publish_node
/rm_revo2_bridge
/move_group
/robot_state_publisher
/rviz
```

### 11.2 检查 action server

```bash
ros2 action info /rm_group_controller/follow_joint_trajectory
ros2 action info /revo2_hand_controller/follow_joint_trajectory
```

应能看到 `/rm_revo2_bridge` 作为 action server。

### 11.3 检查关节状态

```bash
ros2 topic echo /joint_states --once
```

机械臂状态应包含：

```text
joint1 joint2 joint3 joint4 joint5 joint6
```

手部状态应包含：

```text
right_thumb_metacarpal_joint
right_thumb_proximal_joint
right_index_proximal_joint
right_middle_proximal_joint
right_ring_proximal_joint
right_pinky_proximal_joint
```

如果 MoveIt 报：

```text
The complete state of the robot is not yet known. Missing right_...
```

通常表示 `/joint_states` 暂时没有手部关节状态。确认 `rm_revo2_bridge` 已启动，且手部写入成功。

### 11.4 手不动时重点看桥接日志

成功日志通常包含：

```text
hand_backend=sdk
Set_Tool_Voltage(type=3) -> 0
Set_Modbus_Mode(...) -> 0
Write_Single_Register(...) -> 0
Write_Registers -> ret=0
```

如果看到：

```text
Write_Registers failed ret=7: SOCKET_TIME_OUT
```

表示通过 SDK 写末端 485 超时，需要检查 RM65 末端 485 模式、电源、设备 ID、波特率或是否有其他程序占用。

如果看到：

```text
rm_driver Write_Modbus_RTU_Registers failed ret=-21
```

这是旧的 `rm_driver` hand backend 等待结果超时。当前默认已经改为 `sdk`，正常不应优先走这个后端。

### 11.5 机械臂动作很快或一卡一卡

可优先检查：

- `shake_hand.json` 或其他序列文件里的 `sec` 是否太小。
- 目标点之间距离是否太大。
- 是否频繁触发 IK fallback。
- RM65 是否处于报警或限位状态。
- `rm_ws/start_rm65.sh` 是否正常连接。

## 12. 常见启动顺序

完整真机流程建议固定为：

```text
1. 打开 RM65 和 Revo2 供电
2. 终端 1：启动 rm_ws/start_rm65.sh
3. 确认 /joint_states 中有 joint1-joint6
4. 终端 2：启动 rm65_with_revo2_right_moveit.launch.py
5. 确认 /rm_revo2_bridge 存在，hand_backend=sdk
6. 用 revo2_open/revo2_fist 或 tools 脚本测试
7. 再使用 RViz 或序列动作控制整套系统
```

## 13. 相关文档

- Revo2 模型：[`revo2_description/README_CN.md`](revo2_description/README_CN.md)
- Revo2 原始驱动：[`brainco_hardware/brainco_hand_driver/README_CN.md`](brainco_hardware/brainco_hand_driver/README_CN.md)
- Gazebo：[`brainco_gazebo/README_CN.md`](brainco_gazebo/README_CN.md)
- MoveIt：[`brainco_moveit_config/README_CN.md`](brainco_moveit_config/README_CN.md)
- RM65 + Revo2 MoveIt：[`revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md`](revo2_with_rm65_demo/rm65_with_revo2_right_moveit_config/README_CN.md)
- 工具脚本：[`tools/README.md`](tools/README.md)
