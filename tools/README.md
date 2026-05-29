# tools 使用说明

## 1) 单点末端位姿控制
```bash
python3 tools/move_arm_to_pose.py \
  --x -0.20 --y 0.00 --z 0.70 \
  --roll 0 --pitch 0 --yaw 0 \
  --sec 6.0
```

## 2) 连续动作序列（机械臂+手）
```bash
python3 tools/move_arm_pose_sequence.py --sequence-file tools/shake_hand.json
```

可选参数：
- `--hand-mode action|topic`
- `--hand-topic-wait`
- `--ik-timeout`
- `--ik-attempts`

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
