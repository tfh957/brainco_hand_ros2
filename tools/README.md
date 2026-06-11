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

## 5) TCP 实时桥接（可选）
ROS2 端：
```bash
python3 tools/tcp_realtime_bridge_ros2.py
```

发送端：
```bash
python3 tools/tcp_sender_windows.py --host <linux_ip> --port 5000 --fps 10
```
