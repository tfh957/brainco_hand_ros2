# TCP 实时传输（Windows -> Linux ROS2）

## 场景
第一台上位机（Windows）输出手势数据；第二台 Linux 通过 ROS2 控制灵巧手。

## 数据格式
每行一个 JSON（\n 结尾）：
```json
{"hand_positions": [1.0,1.2,1.2,1.2,1.2,1.2], "hand_sec": 0.15}
```

## 启动
Linux:
```bash
python3 tools/tcp_realtime_bridge_ros2.py
```

Windows:
```bash
python3 tools/tcp_sender_windows.py --host <linux_ip> --port 5000 --fps 10
```
