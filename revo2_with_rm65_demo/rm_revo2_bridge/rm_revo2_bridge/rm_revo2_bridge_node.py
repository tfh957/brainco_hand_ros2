#!/usr/bin/env python3
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt16
from trajectory_msgs.msg import JointTrajectory

try:
    from rm_ros_interfaces.msg import Jointpos, Modbusrtuwriteparams, RS485params
except ModuleNotFoundError:
    # fallback: rm_ros_interfaces often exists in rm_ws only
    fallback_paths = [
        "/home/fishros/rmrobot/rm_ws/install/rm_ros_interfaces/local/lib/python3.10/dist-packages",
    ]
    for p in fallback_paths:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.append(p)
    from rm_ros_interfaces.msg import Jointpos, Modbusrtuwriteparams, RS485params


class RmRevo2BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rm_revo2_bridge")
        self.action_cb_group = ReentrantCallbackGroup()
        self._sdk_lock = threading.Lock()
        self._rm_driver_write_event = threading.Event()
        self._rm_driver_write_result: Optional[bool] = None

        self._declare_parameters()

        self.joint_order = [
            "right_thumb_metacarpal_joint",
            "right_thumb_proximal_joint",
            "right_index_proximal_joint",
            "right_middle_proximal_joint",
            "right_ring_proximal_joint",
            "right_pinky_proximal_joint",
        ]
        # device mapping from logical finger order to RM modbus register packing
        self.remap_for_rm = [1, 0, 3, 2, 5, 4]
        self.inverse_remap_for_rm = [1, 0, 3, 2, 5, 4]
        self.last_command_rad = [0.0] * 6

        self.hand_backend = str(self.get_parameter("hand_backend").value).strip().lower()
        if self.hand_backend not in ("auto", "rm_driver", "sdk"):
            self.get_logger().warn(
                f"Unknown hand_backend={self.hand_backend!r}, fallback to sdk"
            )
            self.hand_backend = "sdk"

        self.feedback_enabled = self._as_bool(self.get_parameter("feedback_enabled").value)
        self.arm = None
        if self.hand_backend == "sdk" or self.feedback_enabled:
            self.arm = self._init_rm_arm()

        self.rm_driver_write_pub = None
        self.rm_driver_tool_voltage_pub = None
        self.rm_driver_tool_rs485_pub = None
        self.rm_driver_write_result_sub = None
        if self.hand_backend in ("auto", "rm_driver"):
            self._init_rm_driver_hand_backend()

        self.state_pub_command = self.create_publisher(
            JointState, str(self.get_parameter("state_topic_command").value), 10
        )
        self.state_pub_feedback = self.create_publisher(
            JointState, str(self.get_parameter("state_topic_feedback").value), 10
        )

        self.publish_command_to_joint_states = self._as_bool(
            self.get_parameter("publish_command_to_joint_states").value
        )
        self.state_pub_joint_states = None
        if self.publish_command_to_joint_states:
            self.state_pub_joint_states = self.create_publisher(
                JointState, str(self.get_parameter("joint_states_topic").value), 10
            )
            rate_hz = float(self.get_parameter("publish_hand_state_to_joint_states_rate_hz").value)
            if rate_hz > 0.0:
                self._joint_states_timer = self.create_timer(
                    1.0 / rate_hz, self._publish_last_hand_command_to_joint_states
                )
                self.get_logger().info(
                    f"periodic hand joint_states publishing enabled, rate={rate_hz:.2f}Hz"
                )
            self._publish_last_hand_command_to_joint_states()

        topic = str(self.get_parameter("trajectory_topic").value)
        self.sub = self.create_subscription(JointTrajectory, topic, self.on_trajectory, 10)
        self.get_logger().info(f"rm_revo2_bridge ready, subscribed to: {topic}")

        self.rm_arm_cmd_pub: Optional[object] = None
        self.arm_action_server: Optional[ActionServer] = None
        try:
            self.rm_arm_cmd_pub = self.create_publisher(
                Jointpos, str(self.get_parameter("rm_arm_trajectory_cmd_topic").value), 10
            )
            self.arm_action_server = ActionServer(
                self,
                FollowJointTrajectory,
                "/rm_group_controller/follow_joint_trajectory",
                execute_callback=self.execute_arm_trajectory,
                callback_group=self.action_cb_group,
            )
        except Exception as exc:
            self.get_logger().warn(
                "Failed to initialize arm bridge (rm_ros_interfaces typesupport). "
                f"Hand bridge will continue. detail={exc}"
            )

        self.hand_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/revo2_hand_controller/follow_joint_trajectory",
            execute_callback=self.execute_hand_trajectory,
            callback_group=self.action_cb_group,
        )

        if self.feedback_enabled:
            period = float(self.get_parameter("feedback_timer_period").value)
            self.feedback_timer = self.create_timer(period, self.publish_feedback_state)
            self.get_logger().info(f"feedback enabled, period={period}s")

    def _declare_parameters(self) -> None:
        self.declare_parameter("rm_robot_ip", "192.168.1.18")
        self.declare_parameter("rm_model", "RM65")
        self.declare_parameter("trajectory_topic", "/revo2_hand_controller/joint_trajectory")
        self.declare_parameter("rm_arm_trajectory_cmd_topic", "rm_driver/movej_canfd_cmd")
        self.declare_parameter("tool_port", 1)
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("timeout_100ms", 20)
        self.declare_parameter("device_id", 127)
        self.declare_parameter("init_mode_register", 901)
        self.declare_parameter("init_mode_value", 1)
        self.declare_parameter("position_register", 1070)
        self.declare_parameter("max_joint_rad", 1.7453292519943295)
        self.declare_parameter("robotic_arm_package_path", "/home/fishros/rmrobot/2ndHandDemo/2ndHandDemo")
        self.declare_parameter("set_tool_voltage", True)
        self.declare_parameter("tool_voltage_type", 3)
        self.declare_parameter("hand_backend", "sdk")
        self.declare_parameter("rm_driver_set_tool_voltage_topic", "/rm_driver/set_tool_voltage_cmd")
        self.declare_parameter("rm_driver_set_tool_rs485_topic", "/rm_driver/set_tool_rs485_mode_cmd")
        self.declare_parameter("rm_driver_write_registers_topic", "/rm_driver/write_modbus_rtu_registers_cmd")
        self.declare_parameter("rm_driver_write_registers_result_topic", "/rm_driver/write_modbus_rtu_registers_result")
        self.declare_parameter("rm_driver_modbus_type", 1)
        self.declare_parameter("rm_driver_init_wait_sec", 2.0)
        self.declare_parameter("rm_driver_write_result_timeout_sec", 12.0)
        self.declare_parameter("rm_driver_init_write_mode", True)
        self.declare_parameter("rm_driver_hand_register_pack", "packed_words")
        self.declare_parameter("rm_driver_byte_pair_order", "high_low")

        self.declare_parameter("state_topic_command", "/revo2_bridge/command_joint_states")
        self.declare_parameter("state_topic_feedback", "/revo2_bridge/feedback_joint_states")
        self.declare_parameter("publish_command_to_joint_states", True)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_hand_state_to_joint_states_rate_hz", 15.0)

        self.declare_parameter("feedback_enabled", False)
        self.declare_parameter("feedback_timer_period", 0.1)
        self.declare_parameter("feedback_register_start", 1080)
        self.declare_parameter("feedback_register_num", 3)

        self.declare_parameter("hand_write_retry_count", 1)
        self.declare_parameter("hand_reinit_on_write_fail", True)

        self.declare_parameter("arm_interp_enabled", False)
        self.declare_parameter("arm_cmd_rate_hz", 100.0)
        self.declare_parameter("arm_max_step_rad", 0.02)
        self.declare_parameter("arm_min_segment_dt", 0.02)
        self.declare_parameter("arm_follow", False)

    @staticmethod
    def _as_bool(v) -> bool:
        if isinstance(v, bool):
            return v
        if isinstance(v, (int, float)):
            return v != 0
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "yes", "on")
        return bool(v)

    @staticmethod
    def _ret_code(ret) -> int:
        if isinstance(ret, int):
            return ret
        if isinstance(ret, float):
            return int(ret)
        if isinstance(ret, str):
            s = ret.strip()
            if not s:
                return -999
            head = s.split(":", 1)[0].strip()
            try:
                return int(head)
            except Exception:
                return -999
        return -999

    def _on_rm_driver_write_result(self, msg: Bool) -> None:
        self._rm_driver_write_result = bool(msg.data)
        self._rm_driver_write_event.set()

    def _init_rm_driver_hand_backend(self) -> None:
        write_topic = str(self.get_parameter("rm_driver_write_registers_topic").value)
        write_result_topic = str(self.get_parameter("rm_driver_write_registers_result_topic").value)
        voltage_topic = str(self.get_parameter("rm_driver_set_tool_voltage_topic").value)
        rs485_topic = str(self.get_parameter("rm_driver_set_tool_rs485_topic").value)

        self.rm_driver_write_pub = self.create_publisher(Modbusrtuwriteparams, write_topic, 10)
        self.rm_driver_write_result_sub = self.create_subscription(
            Bool, write_result_topic, self._on_rm_driver_write_result, 10
        )
        self.rm_driver_tool_voltage_pub = self.create_publisher(UInt16, voltage_topic, 10)
        self.rm_driver_tool_rs485_pub = self.create_publisher(RS485params, rs485_topic, 10)

        wait_sec = float(self.get_parameter("rm_driver_init_wait_sec").value)
        deadline = time.monotonic() + max(0.0, wait_sec)
        while time.monotonic() < deadline:
            if self.count_subscribers(write_topic) > 0:
                break
            time.sleep(0.05)

        if self.count_subscribers(write_topic) == 0:
            self.get_logger().warn(
                f"No subscriber on {write_topic}. Is /rm_driver running and sourced?"
            )

        if self._as_bool(self.get_parameter("set_tool_voltage").value):
            voltage_msg = UInt16()
            voltage_msg.data = int(self.get_parameter("tool_voltage_type").value)
            self.rm_driver_tool_voltage_pub.publish(voltage_msg)
            self.get_logger().info(
                f"Published tool voltage via rm_driver: {voltage_topic} data={voltage_msg.data}"
            )

        rs485_msg = RS485params()
        rs485_msg.mode = 0
        rs485_msg.baudrate = int(self.get_parameter("baudrate").value)
        self.rm_driver_tool_rs485_pub.publish(rs485_msg)
        self.get_logger().info(
            f"Published tool RS485 mode via rm_driver: {rs485_topic} mode=0 baudrate={rs485_msg.baudrate}"
        )

        # Let rm_driver process setup messages before writing the Revo2 mode register.
        time.sleep(0.3)
        if self._as_bool(self.get_parameter("rm_driver_init_write_mode").value):
            init_reg = int(self.get_parameter("init_mode_register").value)
            init_val = int(self.get_parameter("init_mode_value").value)
            init_ret = self._write_rm_driver_registers(init_reg, [init_val], wait_result=False)
            self.get_logger().info(
                f"Write init register via rm_driver: address={init_reg}, data={init_val} -> ret={init_ret}"
            )

        self.get_logger().info("Hand backend: rm_driver topic bridge")

    def _init_rm_arm(self):
        sdk_root = str(self.get_parameter("robotic_arm_package_path").value)
        if not os.path.isdir(sdk_root):
            raise RuntimeError(f"robotic_arm_package_path not found: {sdk_root}")
        if sdk_root not in sys.path:
            sys.path.append(sdk_root)

        try:
            from robotic_arm_package.robotic_arm import Arm, RM65, RML63_II  # type: ignore
        except Exception as exc:
            raise RuntimeError(f"Failed to import RM SDK from {sdk_root}: {exc}")

        ip = str(self.get_parameter("rm_robot_ip").value)
        model = str(self.get_parameter("rm_model").value).upper()
        model_id = RM65 if model == "RM65" else RML63_II

        arm = Arm(model_id, ip)

        if self._as_bool(self.get_parameter("set_tool_voltage").value):
            voltage_type = int(self.get_parameter("tool_voltage_type").value)
            ret = arm.Set_Tool_Voltage(type=voltage_type, block=True)
            self.get_logger().info(f"Set_Tool_Voltage(type={voltage_type}) -> {ret}")

        tool_port = int(self.get_parameter("tool_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = int(self.get_parameter("timeout_100ms").value)

        ret_close = arm.Close_Modbus_Mode(port=tool_port, block=True)
        ret_set = arm.Set_Modbus_Mode(port=tool_port, baudrate=baudrate, timeout=timeout, block=True)
        self.get_logger().info(
            f"Close_Modbus_Mode(port={tool_port}) -> {ret_close}, "
            f"Set_Modbus_Mode(baudrate={baudrate}, timeout_100ms={timeout}) -> {ret_set}"
        )

        init_reg = int(self.get_parameter("init_mode_register").value)
        init_val = int(self.get_parameter("init_mode_value").value)
        device_id = int(self.get_parameter("device_id").value)
        ret_init = arm.Write_Single_Register(
            port=tool_port,
            address=init_reg,
            data=init_val,
            device=device_id,
            block=True,
        )
        self.get_logger().info(
            f"Write_Single_Register(port={tool_port}, address={init_reg}, data={init_val}, device={device_id}) -> {ret_init}"
        )

        return arm

    def _ensure_sdk_arm(self) -> None:
        if self.arm is None:
            self.get_logger().warn("Initializing SDK hand backend fallback")
            self.arm = self._init_rm_arm()

    def _reinit_modbus(self) -> None:
        if self.hand_backend in ("auto", "rm_driver"):
            rs485_msg = RS485params()
            rs485_msg.mode = 0
            rs485_msg.baudrate = int(self.get_parameter("baudrate").value)
            if self.rm_driver_tool_rs485_pub is not None:
                self.rm_driver_tool_rs485_pub.publish(rs485_msg)
            init_reg = int(self.get_parameter("init_mode_register").value)
            init_val = int(self.get_parameter("init_mode_value").value)
            init_ret = self._write_rm_driver_registers(init_reg, [init_val], wait_result=True)
            self.get_logger().warn(
                f"rm_driver Modbus reinit published: rs485 mode=0 baudrate={rs485_msg.baudrate}, init_ret={init_ret}"
            )
            return

        tool_port = int(self.get_parameter("tool_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = int(self.get_parameter("timeout_100ms").value)
        init_reg = int(self.get_parameter("init_mode_register").value)
        init_val = int(self.get_parameter("init_mode_value").value)
        device_id = int(self.get_parameter("device_id").value)

        ret_close = self.arm.Close_Modbus_Mode(port=tool_port, block=True)
        ret_set = self.arm.Set_Modbus_Mode(port=tool_port, baudrate=baudrate, timeout=timeout, block=True)
        ret_init = self.arm.Write_Single_Register(
            port=tool_port,
            address=init_reg,
            data=init_val,
            device=device_id,
            block=True,
        )
        self.get_logger().warn(
            "Modbus reinit done: "
            f"close={ret_close}, set={ret_set}, init={ret_init}"
        )

    def _write_rm_driver_registers(
        self,
        address: int,
        data: List[int],
        wait_result: bool = True,
        num: Optional[int] = None,
    ) -> int:
        if self.rm_driver_write_pub is None:
            return -20

        msg = Modbusrtuwriteparams()
        msg.address = int(address)
        msg.device = int(self.get_parameter("device_id").value)
        msg.type = int(self.get_parameter("rm_driver_modbus_type").value)
        msg.num = len(data) if num is None else int(num)
        msg.data = [int(v) for v in data]

        self._rm_driver_write_result = None
        self._rm_driver_write_event.clear()
        self.rm_driver_write_pub.publish(msg)

        if not wait_result:
            return 0

        timeout = float(self.get_parameter("rm_driver_write_result_timeout_sec").value)
        if not self._rm_driver_write_event.wait(max(0.1, timeout)):
            return -21
        return 0 if self._rm_driver_write_result else -22

    def _pack_hand_register_data_for_rm_driver(self, adjusted: List[int]) -> tuple[List[int], int]:
        mode = str(self.get_parameter("rm_driver_hand_register_pack").value).strip().lower()
        if mode in ("raw", "raw_bytes", "bytes"):
            return [int(v) for v in adjusted], 3

        if len(adjusted) != 6:
            return [int(v) for v in adjusted], len(adjusted)

        order = str(self.get_parameter("rm_driver_byte_pair_order").value).strip().lower()
        words: List[int] = []
        for i in range(0, 6, 2):
            first = max(0, min(255, int(adjusted[i])))
            second = max(0, min(255, int(adjusted[i + 1])))
            if order in ("low_high", "little", "little_endian"):
                word = (second << 8) | first
            else:
                word = (first << 8) | second
            words.append(word)
        return words, len(words)

    def _write_hand_registers_sdk(self, adjusted: List[int]) -> int:
        self._ensure_sdk_arm()

        tool_port = int(self.get_parameter("tool_port").value)
        position_reg = int(self.get_parameter("position_register").value)
        device_id = int(self.get_parameter("device_id").value)
        retries = int(self.get_parameter("hand_write_retry_count").value)
        if retries < 0:
            retries = 0
        do_reinit = self._as_bool(self.get_parameter("hand_reinit_on_write_fail").value)

        last_ret = -1
        last_code = -1
        for attempt in range(retries + 1):
            with self._sdk_lock:
                last_ret = self.arm.Write_Registers(
                    port=tool_port,
                    address=position_reg,
                    num=3,
                    single_data=adjusted,
                    device=device_id,
                    block=True,
                )
            last_code = self._ret_code(last_ret)
            if last_code == 0:
                return 0

            self.get_logger().warn(
                f"SDK Write_Registers failed ret={last_ret}, attempt={attempt + 1}/{retries + 1}"
            )
            if do_reinit and attempt < retries:
                try:
                    with self._sdk_lock:
                        # Force SDK-style reinitialization even when hand_backend is auto.
                        original_backend = self.hand_backend
                        self.hand_backend = "sdk"
                        self._reinit_modbus()
                        self.hand_backend = original_backend
                except Exception as exc:
                    self.get_logger().warn(f"SDK Modbus reinit exception: {exc}")
                    self.hand_backend = original_backend
            if attempt < retries:
                time.sleep(0.05)

        return last_code

    def _write_hand_registers(self, adjusted: List[int]) -> int:
        if self.hand_backend in ("auto", "rm_driver"):
            position_reg = int(self.get_parameter("position_register").value)
            retries = int(self.get_parameter("hand_write_retry_count").value)
            if retries < 0:
                retries = 0
            do_reinit = self._as_bool(self.get_parameter("hand_reinit_on_write_fail").value)

            ret = -1
            for attempt in range(retries + 1):
                data, num = self._pack_hand_register_data_for_rm_driver(adjusted)
                ret = self._write_rm_driver_registers(position_reg, data, wait_result=True, num=num)
                if ret == 0:
                    return 0
                self.get_logger().warn(
                    f"rm_driver Write_Modbus_RTU_Registers failed ret={ret}, "
                    f"attempt={attempt + 1}/{retries + 1}, num={num}, data={data}"
                )
                if do_reinit and attempt < retries:
                    self._reinit_modbus()
                    time.sleep(0.05)
            if self.hand_backend == "rm_driver":
                return ret

            self.get_logger().warn(
                f"rm_driver hand backend failed ret={ret}; falling back to SDK backend"
            )
            return self._write_hand_registers_sdk(adjusted)

        return self._write_hand_registers_sdk(adjusted)

    def _rad_to_percent(self, rad: float) -> int:
        max_joint_rad = float(self.get_parameter("max_joint_rad").value)
        if max_joint_rad <= 0.0:
            max_joint_rad = math.radians(100.0)
        p = int(round((rad / max_joint_rad) * 100.0))
        return max(0, min(100, p))

    def _percent_to_rad(self, percent: int) -> float:
        max_joint_rad = float(self.get_parameter("max_joint_rad").value)
        if max_joint_rad <= 0.0:
            max_joint_rad = math.radians(100.0)
        return (float(percent) / 100.0) * max_joint_rad

    def _publish_joint_state(self, publisher, positions_rad: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_order)
        msg.position = list(positions_rad)
        publisher.publish(msg)

    def _publish_last_hand_command_to_joint_states(self) -> None:
        if self.state_pub_joint_states is None:
            return
        self._publish_joint_state(self.state_pub_joint_states, self.last_command_rad)

    def publish_feedback_state(self) -> None:
        if self.arm is None:
            return

        tool_port = int(self.get_parameter("tool_port").value)
        device_id = int(self.get_parameter("device_id").value)
        start = int(self.get_parameter("feedback_register_start").value)
        num = int(self.get_parameter("feedback_register_num").value)

        try:
            ret, data = self.arm.Read_Multiple_Input_Registers(
                port=tool_port, address=start, num=num, device=device_id
            )
        except Exception as exc:
            self.get_logger().warn(f"feedback read exception: {exc}")
            return

        code = self._ret_code(ret)
        if code != 0:
            self.get_logger().warn(f"feedback read failed, ret={ret}")
            return

        if len(data) < 6:
            self.get_logger().warn(f"feedback read size invalid: {len(data)}")
            return

        mapped = [max(0, min(100, int(v))) for v in data[:6]]
        ordered_percent = [mapped[i] for i in self.inverse_remap_for_rm]
        ordered_rad = [self._percent_to_rad(v) for v in ordered_percent]
        self._publish_joint_state(self.state_pub_feedback, ordered_rad)

    def on_trajectory(self, msg: JointTrajectory) -> int:
        if not msg.points:
            self.get_logger().warn("Ignore trajectory: no points.")
            return -1

        point = msg.points[-1]
        if len(point.positions) != len(msg.joint_names):
            self.get_logger().warn("Ignore trajectory: positions size does not match joint_names size.")
            return -2

        target_map: Dict[str, float] = {}
        for i, name in enumerate(msg.joint_names):
            target_map[name] = point.positions[i]

        ordered: List[int] = []
        ordered_rad: List[float] = []
        missing = []
        for name in self.joint_order:
            if name not in target_map:
                missing.append(name)
                continue
            rad = float(target_map[name])
            ordered_rad.append(rad)
            ordered.append(self._rad_to_percent(rad))

        if missing:
            self.get_logger().warn(f"Ignore trajectory: missing joints: {missing}")
            return -3

        adjusted = [ordered[i] for i in self.remap_for_rm]

        ret = self._write_hand_registers(adjusted)
        if ret == 0:
            self.last_command_rad = ordered_rad
            self._publish_joint_state(self.state_pub_command, ordered_rad)
            if self.state_pub_joint_states is not None:
                self._publish_joint_state(self.state_pub_joint_states, ordered_rad)
        else:
            self.get_logger().warn(
                f"Skip publishing command/joint_states because hand write failed (ret={ret})"
            )
        self.get_logger().info(f"Write_Registers -> ret={ret}, percent={ordered}, mapped={adjusted}")
        return ret

    def execute_hand_trajectory(self, goal_handle):
        goal = goal_handle.request
        if not goal.trajectory.points:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        ret = self.on_trajectory(goal.trajectory)
        if ret != 0:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.abort()
            return result

        end_t = goal.trajectory.points[-1].time_from_start
        wait_s = float(end_t.sec) + float(end_t.nanosec) / 1e9
        if wait_s > 0.0:
            time.sleep(wait_s)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def _publish_arm_cmd(self, joints: List[float], follow: bool) -> int:
        if self.rm_arm_cmd_pub is None:
            return -1

        msg = Jointpos()
        msg.joint = [float(v) for v in joints]
        msg.follow = bool(follow)
        msg.expand = 0.0
        msg.dof = 6
        self.rm_arm_cmd_pub.publish(msg)
        self.get_logger().info(f"Arm cmd publish (first point): {msg.joint}")
        return 0

    def execute_arm_trajectory(self, goal_handle):
        if self.rm_arm_cmd_pub is None:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        goal = goal_handle.request
        traj = goal.trajectory
        if not traj.points or not traj.joint_names:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        expected = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        if any(n not in name_to_idx for n in expected):
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            goal_handle.abort()
            return result

        interp_enabled = self._as_bool(self.get_parameter("arm_interp_enabled").value)
        cmd_rate_hz = float(self.get_parameter("arm_cmd_rate_hz").value)
        if cmd_rate_hz <= 0.0:
            cmd_rate_hz = 50.0
        max_step_rad = float(self.get_parameter("arm_max_step_rad").value)
        if max_step_rad <= 0.0:
            max_step_rad = 0.02
        min_segment_dt = float(self.get_parameter("arm_min_segment_dt").value)
        if min_segment_dt <= 0.0:
            min_segment_dt = 0.01
        follow = self._as_bool(self.get_parameter("arm_follow").value)

        prev_joints: Optional[List[float]] = None
        prev_t = 0.0

        for i, point in enumerate(traj.points):
            if len(point.positions) != len(traj.joint_names):
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
                return result

            target = [float(point.positions[name_to_idx[n]]) for n in expected]
            t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
            dt = max(0.0, t - prev_t)

            if prev_joints is None:
                self._publish_arm_cmd(target, follow)
                if dt > 0.0:
                    time.sleep(dt)
                prev_joints = list(target)
                prev_t = t
                continue

            if interp_enabled and dt > 0.0:
                max_delta = max(abs(target[j] - prev_joints[j]) for j in range(6))
                seg_by_step = int(math.ceil(max_delta / max_step_rad)) if max_step_rad > 1e-9 else 1
                seg_by_rate = int(math.ceil(dt * cmd_rate_hz))
                segments = max(1, seg_by_step, seg_by_rate)
                seg_dt = max(min_segment_dt, dt / segments)

                for s in range(1, segments + 1):
                    alpha = float(s) / float(segments)
                    cmd = [prev_joints[j] + alpha * (target[j] - prev_joints[j]) for j in range(6)]
                    self._publish_arm_cmd(cmd, follow)
                    time.sleep(seg_dt)
            else:
                self._publish_arm_cmd(target, follow)
                if dt > 0.0:
                    time.sleep(max(min_segment_dt, dt))

            prev_joints = list(target)
            prev_t = t

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RmRevo2BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
