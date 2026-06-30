# tools 使用说明

## 1) 单点末端位姿控制
```bash
python3 tools/move_arm_to_pose.py \
  --x -0.20 --y 0.00 --z 0.70 \
  --roll 0 --pitch 0 --yaw 0 \
  --sec 6.0
```

## 2) 连续动作序列（机械臂+手）
严格时序模式，手部使用 action，会等待桥接节点返回执行结果：
```bash
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json
```

脚本默认启用 IK 关节跳变保护：如果某个目标位姿解出来的任意关节相对当前状态跳变超过 `4rad`，会先停止，避免直接冲向异常姿态。确认安全后可调整：
```bash
python3 tools/move_arm_pose_sequence.py \
  --sequence-file tools/shake_hand.json \
  --max-joint-jump 1.5
```

只调试机械臂位姿时，可以先跳过手部命令，避免 485 超时干扰机械臂排查：
```bash
python3 tools/move_arm_pose_sequence.py \
  --sequence-file tools/shake_hand.json \
  --skip-hand
```

如果 485/Modbus 偶发超时，先用 topic 模式验证动作链路，程序会按 `hand_sec` 等待后继续下一步：
```bash
python3 tools/move_arm_pose_sequence.py \
  --sequence-file tools/shake_hand.json \
  --hand-mode topic \
  --hand-topic-wait
```

可选参数：
- `--hand-mode action|topic`
- `--hand-topic-wait`
- `--hand-result-timeout-min`
- `--ik-timeout`
- `--ik-attempts`
- `--max-joint-jump`
- `--skip-hand`

## 3) 预设动作快捷命令
```bash
source tools/revo2_cmds.sh
revo2_open
revo2_fist
```

## 4) 演示脚本
```bash
python3 tools/sequence_demo.py
```

## 5) TCP 实时桥接（Windows/LabVIEW -> Linux ROS2）

`tcp_realtime_bridge_ros2.py` 用于把 Windows 端通过 TCP 发来的灵巧手关节目标转成 ROS2 话题。

当前脚本只控制 Revo2 灵巧手，不处理 RM65 机械臂末端位姿。

### 5.1 通讯链路

```text
Windows / LabVIEW
  TCP Client
  发送一行 JSON + 换行符
        |
        v
Linux / tcp_realtime_bridge_ros2.py
  TCP Server: 0.0.0.0:5000
  发布 JointTrajectory
        |
        v
/revo2_hand_controller/joint_trajectory
        |
        v
rm_revo2_bridge / Revo2 灵巧手
```

Linux 端显示 `0.0.0.0:5000` 是正常的，表示监听本机所有网卡。Windows 端连接时不要填写 `0.0.0.0`，要填写 Linux 实际 IP，例如 WiFi IP `10.xxx.xxx.xxx`。

### 5.2 启动完整控制系统

终端 1：启动 RM65 驱动。

```bash
cd ~/rmrobot/rm_ws
source install/setup.bash
./start_rm65.sh
```

终端 2：启动 MoveIt、RViz 和 Revo2 桥接。

```bash
cd ~/rmrobot/brainco_ws
source install/setup.bash
ros2 launch rm65_with_revo2_right_moveit_config rm65_with_revo2_right_moveit.launch.py
```

终端 3：启动 TCP bridge。

```bash
cd ~/rmrobot/brainco_ws/src/src/brainco_hand_ros2
source ~/rmrobot/brainco_ws/install/setup.bash
python3 tools/tcp_realtime_bridge_ros2.py
```

期望日志：

```text
TCP bridge listening on 0.0.0.0:5000, pub -> /revo2_hand_controller/joint_trajectory
```

### 5.3 Windows/LabVIEW 发送格式

TCP bridge 按“每行一个 JSON”解析数据，所以每条消息末尾必须带真正的换行符 `LF`。

JSON 示例：

```json
{"hand_positions":[1.0,1.2,1.2,1.2,1.2,1.2],"hand_sec":1.0}
```

实际 TCP 发送内容需要是：

```text
{"hand_positions":[1.0,1.2,1.2,1.2,1.2,1.2],"hand_sec":1.0}\n
```

字段说明：

- `hand_positions`：长度必须为 6，对应 6 个手部关节目标，单位为 rad。
- `hand_sec`：该条轨迹的期望执行时间，单位为秒；未提供时默认 `0.2`。
- 关节顺序固定为 `right_thumb_metacarpal_joint`、`right_thumb_proximal_joint`、`right_index_proximal_joint`、`right_middle_proximal_joint`、`right_ring_proximal_joint`、`right_pinky_proximal_joint`。

LabVIEW 中不要把普通字符串 `\n` 当换行符发送。推荐在程序框图里使用：

```text
JSON 字符串 + End of Line Constant
```

或者把字符串控件切换到 `\ Codes Display` 后再输入 `\n`。

### 5.4 Python 发送端测试

如果暂时不用 LabVIEW，可以在 Windows 或另一台电脑上用示例脚本测试：

```bash
python3 tools/tcp_sender_windows.py --host <linux_ip> --port 5000 --fps 10
```

也可以在 Linux 本机测试 TCP bridge 是否会发布 ROS2 话题：

```bash
printf '{"hand_positions":[1.0,1.2,1.2,1.2,1.2,1.2],"hand_sec":1.0}\n' | nc 127.0.0.1 5000
```

### 5.5 ROS2 侧验证

查看 TCP bridge 是否发布了消息：

```bash
ros2 topic echo /revo2_hand_controller/joint_trajectory
```

查看是否有控制节点订阅：

```bash
ros2 topic info /revo2_hand_controller/joint_trajectory
```

正常情况下应能看到：

```text
Publisher count: 1
Subscription count: 1
```

如果 `Publisher count: 1`、`Subscription count: 0`，说明 TCP bridge 在发布，但 Revo2 桥接或控制系统没有订阅该话题。

### 5.6 常见问题

- TCP 日志只显示 `client connected` 和 `client disconnected`，但 `ros2 topic echo` 没消息：通常是 LabVIEW 没有发送真正的换行符，或者 JSON 字段拼写错误。
- Linux `nc -lv 5000` 看到字面量 `\n`：说明 LabVIEW 发的是反斜杠和字母 n，不是真正换行符。
- 发送 `[0,0,0,0,0,0]` 但手没动：这通常是张开姿态，如果手已经张开就看不出动作。可在安全确认后测试 `[1.0,1.2,1.2,1.2,1.2,1.2]`。
- `hand_positions` 数组不是 6 个数：脚本会直接忽略该帧。
- JSON 格式错误：当前脚本会静默忽略该帧，不会发布 ROS2 消息。

### 5.7 实时发送注意事项

当前脚本收到一条合法 JSON 后会立即发布一条 `JointTrajectory`。如果 Windows 高频连续发送，新的目标会不断覆盖上一条控制意图，手可能还没到达上一帧目标就收到下一帧目标。

建议初期参数：

- 发送频率先用 `10 Hz`，稳定后再尝试 `20 Hz` 或 `30 Hz`。
- `hand_sec` 可设置为接近发送周期，例如 `0.05 ~ 0.15`。
- LabVIEW 端尽量发送最新目标，不要堆积历史目标。
- 如果要长期做实时手势复现，建议后续增加限频、最新帧缓存、看门狗、断线停止和输入限幅逻辑。
