#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_ACTION = "/rm_group_controller/follow_joint_trajectory"
HAND_ACTION = "/revo2_hand_controller/follow_joint_trajectory"
HAND_TOPIC = "/revo2_hand_controller/joint_trajectory"
IK_SERVICE = "/compute_ik"

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HAND_JOINTS = [
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class MoveArmPoseSequence(Node):
    def __init__(self):
        super().__init__("move_arm_pose_sequence")
        self.ik_client = self.create_client(GetPositionIK, IK_SERVICE)
        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.hand_client = ActionClient(self, FollowJointTrajectory, HAND_ACTION)
        self.hand_topic_pub = self.create_publisher(JointTrajectory, HAND_TOPIC, 10)

        self.last_arm_joint_state: Optional[JointState] = None
        self.last_commanded_arm_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

    @staticmethod
    def _arm_only_joint_state(msg: JointState) -> Optional[JointState]:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in name_to_idx for n in ARM_JOINTS):
            return None
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(msg.position[name_to_idx[n]]) for n in ARM_JOINTS]
        return js

    def _on_joint_state(self, msg: JointState):
        js = self._arm_only_joint_state(msg)
        if js is not None:
            self.last_arm_joint_state = js

    def _wait_for_arm_joint_state(self, timeout_sec: float = 2.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while self.last_arm_joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    @staticmethod
    def _joint_positions(joint_state: JointState, joint_names: List[str]) -> List[float]:
        name_to_idx = {n: i for i, n in enumerate(joint_state.name)}
        return [float(joint_state.position[name_to_idx[n]]) for n in joint_names]

    def _max_joint_delta(self, target_state: JointState, reference_state: JointState) -> float:
        target = self._joint_positions(target_state, ARM_JOINTS)
        reference = self._joint_positions(reference_state, ARM_JOINTS)
        return max(abs(target[i] - reference[i]) for i in range(len(ARM_JOINTS)))

    def _ik_seed_joint_state(self) -> Optional[JointState]:
        # Prefer the last solution this script commanded. Live /joint_states can lag or
        # briefly contain only hand joints while hand-only steps are running.
        return self.last_commanded_arm_joint_state or self.last_arm_joint_state

    def _solve_ik(
        self,
        group_name: str,
        ik_link_name: str,
        frame_id: str,
        xyz: Tuple[float, float, float],
        quat: Tuple[float, float, float, float],
        ik_timeout_sec: float,
        ik_attempts: int,
        retry_without_collisions: bool,
    ) -> JointState:
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/compute_ik service not available")

        seed_state = self._ik_seed_joint_state()
        last_code = None
        best_solution = None
        best_delta = float("inf")
        collision_modes = [True]
        if retry_without_collisions:
            collision_modes.append(False)

        for avoid_collisions in collision_modes:
            if not avoid_collisions:
                self.get_logger().warn(
                    "Collision-aware IK failed; retrying IK without collision checking. "
                    "Joint jump limit is still enforced before execution."
                )

            for _ in range(max(1, ik_attempts)):
                req = GetPositionIK.Request()
                req.ik_request = PositionIKRequest()
                req.ik_request.group_name = group_name
                req.ik_request.ik_link_name = ik_link_name
                req.ik_request.pose_stamped.header.frame_id = frame_id
                req.ik_request.pose_stamped.pose.position.x = float(xyz[0])
                req.ik_request.pose_stamped.pose.position.y = float(xyz[1])
                req.ik_request.pose_stamped.pose.position.z = float(xyz[2])
                req.ik_request.pose_stamped.pose.orientation.x = float(quat[0])
                req.ik_request.pose_stamped.pose.orientation.y = float(quat[1])
                req.ik_request.pose_stamped.pose.orientation.z = float(quat[2])
                req.ik_request.pose_stamped.pose.orientation.w = float(quat[3])
                req.ik_request.timeout.sec = int(ik_timeout_sec)
                req.ik_request.timeout.nanosec = int((ik_timeout_sec - int(ik_timeout_sec)) * 1e9)
                req.ik_request.avoid_collisions = avoid_collisions
                if seed_state is not None:
                    req.ik_request.robot_state.joint_state = seed_state

                fut = self.ik_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=ik_timeout_sec + 1.0)
                if not fut.done() or fut.result() is None:
                    continue
                resp = fut.result()
                if resp.error_code.val == MoveItErrorCodes.SUCCESS:
                    if seed_state is None:
                        return resp.solution.joint_state
                    delta = self._max_joint_delta(resp.solution.joint_state, seed_state)
                    if delta < best_delta:
                        best_delta = delta
                        best_solution = resp.solution.joint_state
                last_code = resp.error_code.val

            if best_solution is not None:
                return best_solution

        if last_code is None:
            raise RuntimeError("IK timeout")
        raise RuntimeError(f"IK failed, MoveItErrorCode={last_code}")

    @staticmethod
    def _build_arm_goal(joint_state: JointState, sec: float) -> FollowJointTrajectory.Goal:
        name_to_idx = {n: i for i, n in enumerate(joint_state.name)}
        if any(n not in name_to_idx for n in ARM_JOINTS):
            raise RuntimeError(f"IK result missing joints: {ARM_JOINTS}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(joint_state.position[name_to_idx[n]]) for n in ARM_JOINTS]
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        goal.trajectory.points = [point]
        return goal

    @staticmethod
    def _build_hand_goal(hand_positions: List[float], sec: float) -> FollowJointTrajectory.Goal:
        if len(hand_positions) != 6:
            raise RuntimeError("hand_positions must be 6 values")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(HAND_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in hand_positions]
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        goal.trajectory.points = [point]
        return goal

    def _publish_hand_topic(self, hand_positions: List[float], sec: float):
        traj = JointTrajectory()
        traj.joint_names = list(HAND_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in hand_positions]
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        traj.points = [point]
        self.hand_topic_pub.publish(traj)

    def _send_goal_get_handle(
        self,
        client: ActionClient,
        goal: FollowJointTrajectory.Goal,
        label: str,
        send_goal_timeout_sec: float,
    ):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{label} action server not available")
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=send_goal_timeout_sec)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"{label} send goal timeout")
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError(f"{label} goal rejected")
        return handle

    def _wait_goal_result(self, handle, label: str, sec: float, min_result_timeout_sec: float):
        fut = handle.get_result_async()
        timeout_sec = max(min_result_timeout_sec, sec + 5.0)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if not fut.done() or fut.result() is None:
            handle.cancel_goal_async()
            raise RuntimeError(f"{label} execution timeout ({timeout_sec:.1f}s)")
        result = fut.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"{label} action failed, error_code={result.error_code}")

    def _send_and_wait(
        self,
        client: ActionClient,
        goal: FollowJointTrajectory.Goal,
        label: str,
        sec: float,
        min_result_timeout_sec: float = 10.0,
        send_goal_timeout_sec: float = 5.0,
    ):
        handle = self._send_goal_get_handle(client, goal, label, send_goal_timeout_sec)
        self._wait_goal_result(handle, label, sec, min_result_timeout_sec)

    @staticmethod
    def _step_quat(step: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
        if all(k in step for k in ("qx", "qy", "qz", "qw")):
            return (float(step["qx"]), float(step["qy"]), float(step["qz"]), float(step["qw"]))
        if "rpy" in step:
            rpy = step["rpy"]
            return rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        if all(k in step for k in ("roll", "pitch", "yaw")):
            return rpy_to_quat(float(step["roll"]), float(step["pitch"]), float(step["yaw"]))
        return None

    def _execute_hand_step(
        self,
        step_name: str,
        hand_positions: List[float],
        hand_sec: float,
        hand_mode: str,
        hand_result_timeout_min_sec: float,
    ):
        self.get_logger().info(
            f"{step_name}: hand={hand_positions}, hand_sec={hand_sec:.2f}"
        )
        if hand_mode == "action":
            hand_goal = self._build_hand_goal(hand_positions, hand_sec)
            self._send_and_wait(
                self.hand_client,
                hand_goal,
                f"{step_name} hand",
                hand_sec,
                min_result_timeout_sec=hand_result_timeout_min_sec,
            )
        else:
            self._publish_hand_topic(hand_positions, hand_sec)

    def run_sequence(
        self,
        steps: List[Dict[str, Any]],
        group_name: str,
        ik_link_name: str,
        frame_id: str,
        ik_timeout_sec: float,
        ik_attempts: int,
        hand_result_timeout_min_sec: float,
        hand_mode: str,
        hand_topic_wait: bool,
        arm_send_goal_timeout_sec: float,
        max_joint_jump: float,
        skip_hand: bool,
        retry_ik_without_collisions: bool,
    ):
        total = len(steps)
        for i, step in enumerate(steps, start=1):
            name = str(step.get("name", f"step_{i}"))

            has_arm = all(k in step for k in ("x", "y", "z"))
            has_hand = "hand_positions" in step
            if has_hand and skip_hand:
                self.get_logger().info(f"[{i}/{total}] {name}: hand skipped (--skip-hand)")
                has_hand = False

            arm_goal = None
            arm_sec = float(step.get("sec", 3.0))
            hand_sec = float(step.get("hand_sec", 1.0))

            if has_arm:
                self._wait_for_arm_joint_state()
                quat = self._step_quat(step)
                if quat is None:
                    raise RuntimeError(f"{name}: arm step requires quaternion or rpy")
                xyz = (float(step["x"]), float(step["y"]), float(step["z"]))
                self.get_logger().info(
                    f"[{i}/{total}] {name}: arm xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}), "
                    f"q=({quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f}), arm_sec={arm_sec:.2f}"
                )
                js = self._solve_ik(
                    group_name=group_name,
                    ik_link_name=ik_link_name,
                    frame_id=frame_id,
                    xyz=xyz,
                    quat=quat,
                    ik_timeout_sec=ik_timeout_sec,
                    ik_attempts=ik_attempts,
                    retry_without_collisions=retry_ik_without_collisions,
                )
                if self.last_arm_joint_state is not None:
                    joint_delta = self._max_joint_delta(js, self.last_arm_joint_state)
                    self.get_logger().info(
                        f"{name}: ik_joints={[round(v, 4) for v in self._joint_positions(js, ARM_JOINTS)]}, "
                        f"max_joint_delta={joint_delta:.3f}rad"
                    )
                    if max_joint_jump > 0.0 and joint_delta > max_joint_jump:
                        raise RuntimeError(
                            f"{name}: IK solution joint jump too large "
                            f"({joint_delta:.3f}rad > {max_joint_jump:.3f}rad). "
                            "Check target pose/rpy or increase --max-joint-jump after confirming safety."
                        )
                arm_goal = self._build_arm_goal(js, arm_sec)

            if has_hand:
                hand_positions = [float(v) for v in step["hand_positions"]]
            else:
                hand_positions = []

            order = str(step.get("order", "auto")).lower()
            if "parallel" in step:
                order = "parallel" if bool(step["parallel"]) else "arm_then_hand"
            gap_sec = float(step.get("gap_sec", 0.0))

            if has_arm and has_hand:
                if order == "parallel":
                    if hand_mode == "action":
                        arm_handle = self._send_goal_get_handle(
                            self.arm_client,
                            arm_goal,
                            f"{name} arm",
                            arm_send_goal_timeout_sec,
                        )
                        hand_goal = self._build_hand_goal(hand_positions, hand_sec)
                        hand_handle = self._send_goal_get_handle(
                            self.hand_client,
                            hand_goal,
                            f"{name} hand",
                            5.0,
                        )
                        self._wait_goal_result(arm_handle, f"{name} arm", arm_sec, 10.0)
                        self.last_commanded_arm_joint_state = self._arm_only_joint_state(js)
                        self._wait_goal_result(
                            hand_handle,
                            f"{name} hand",
                            hand_sec,
                            hand_result_timeout_min_sec,
                        )
                    else:
                        self._send_and_wait(
                            self.arm_client,
                            arm_goal,
                            f"{name} arm",
                            arm_sec,
                            send_goal_timeout_sec=arm_send_goal_timeout_sec,
                        )
                        self.last_commanded_arm_joint_state = self._arm_only_joint_state(js)
                        self._execute_hand_step(
                            step_name=f"[{i}/{total}] {name}",
                            hand_positions=hand_positions,
                            hand_sec=hand_sec,
                            hand_mode=hand_mode,
                            hand_result_timeout_min_sec=hand_result_timeout_min_sec,
                        )
                        if hand_topic_wait:
                            time.sleep(max(0.0, hand_sec))
                elif order == "hand_then_arm":
                    self._execute_hand_step(
                        step_name=f"[{i}/{total}] {name}",
                        hand_positions=hand_positions,
                        hand_sec=hand_sec,
                        hand_mode=hand_mode,
                        hand_result_timeout_min_sec=hand_result_timeout_min_sec,
                    )
                    if hand_topic_wait:
                        time.sleep(max(0.0, hand_sec))
                    if gap_sec > 0.0:
                        time.sleep(gap_sec)
                    self._send_and_wait(
                        self.arm_client,
                        arm_goal,
                        f"{name} arm",
                        arm_sec,
                        send_goal_timeout_sec=arm_send_goal_timeout_sec,
                    )
                    self.last_commanded_arm_joint_state = self._arm_only_joint_state(js)
                else:
                    # auto / arm_then_hand
                    self._send_and_wait(
                        self.arm_client,
                        arm_goal,
                        f"{name} arm",
                        arm_sec,
                        send_goal_timeout_sec=arm_send_goal_timeout_sec,
                    )
                    self.last_commanded_arm_joint_state = self._arm_only_joint_state(js)
                    if gap_sec > 0.0:
                        time.sleep(gap_sec)
                    self._execute_hand_step(
                        step_name=f"[{i}/{total}] {name}",
                        hand_positions=hand_positions,
                        hand_sec=hand_sec,
                        hand_mode=hand_mode,
                        hand_result_timeout_min_sec=hand_result_timeout_min_sec,
                    )
                    if hand_topic_wait:
                        time.sleep(max(0.0, hand_sec))
            elif has_arm:
                self._send_and_wait(
                    self.arm_client,
                    arm_goal,
                    f"{name} arm",
                    arm_sec,
                    send_goal_timeout_sec=arm_send_goal_timeout_sec,
                )
                self.last_commanded_arm_joint_state = self._arm_only_joint_state(js)
            elif has_hand:
                self.get_logger().info(f"[{i}/{total}] {name}")
                self._execute_hand_step(
                    step_name=f"{name}",
                    hand_positions=hand_positions,
                    hand_sec=hand_sec,
                    hand_mode=hand_mode,
                    hand_result_timeout_min_sec=hand_result_timeout_min_sec,
                )
                if hand_topic_wait:
                    time.sleep(max(0.0, hand_sec))
            else:
                self.get_logger().warn(f"[{i}/{total}] {name}: empty step, skip")


def build_parser():
    p = argparse.ArgumentParser(description="Run mixed arm-pose + hand sequence")
    p.add_argument("--sequence-file", default="", help="JSON file path")

    p.add_argument("--group", default="rm65")
    p.add_argument("--ik-link", default="right_base_link")
    p.add_argument("--frame", default="base_link")
    p.add_argument("--ik-timeout", type=float, default=0.5)
    p.add_argument("--ik-attempts", type=int, default=20)

    p.add_argument(
        "--hand-result-timeout-min",
        type=float,
        default=40.0,
        help="minimum wait time for hand action result (s); Modbus retry/reinit can take a while",
    )
    p.add_argument(
        "--hand-mode",
        choices=["action", "topic"],
        default="action",
        help="hand control mode: topic (non-blocking, recommended) or action (strict sync)",
    )
    p.add_argument(
        "--hand-topic-wait",
        action="store_true",
        help="when hand-mode=topic, wait hand_sec after publishing each hand step",
    )
    p.add_argument(
        "--arm-send-goal-timeout",
        type=float,
        default=20.0,
        help="timeout for arm action goal acceptance (s)",
    )
    p.add_argument(
        "--max-joint-jump",
        type=float,
        default=4,
        help="stop before execution if any IK joint differs from current state by more than this radian value; <=0 disables",
    )
    p.add_argument(
        "--skip-hand",
        action="store_true",
        help="ignore hand_positions in the sequence; useful when isolating arm IK/motion from 485 issues",
    )
    p.add_argument(
        "--no-ik-collision-fallback",
        action="store_true",
        help="do not retry IK without collision checking after collision-aware IK fails",
    )
    return p


def default_steps() -> List[Dict[str, Any]]:
    return [
        {
            "name": "arm_pregrasp",
            "x": 0.32,
            "y": 0.00,
            "z": 0.30,
            "rpy": [0.0, 0.0, 0.0],
            "sec": 6.0,
        },
        {
            "name": "hand_open",
            "hand_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "hand_sec": 1.0,
        },
        {
            "name": "move_left_and_fist",
            "x": 0.32,
            "y": 0.08,
            "z": 0.30,
            "rpy": [0.0, 0.0, 0.0],
            "sec": 4.0,
            "hand_positions": [1.0, 1.2, 1.2, 1.2, 1.2, 1.2],
            "hand_sec": 2.0,
            "parallel": True,
        },
    ]


def load_steps(path: str) -> List[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    steps = data.get("steps", data) if isinstance(data, dict) else data
    if not isinstance(steps, list) or not steps:
        raise ValueError("sequence file must be a non-empty list or contain a 'steps' list")
    return steps


def main():
    args = build_parser().parse_args()
    rclpy.init(args=sys.argv)
    node = MoveArmPoseSequence()
    try:
        steps = load_steps(args.sequence_file) if args.sequence_file else default_steps()
        node.run_sequence(
            steps=steps,
            group_name=args.group,
            ik_link_name=args.ik_link,
            frame_id=args.frame,
            ik_timeout_sec=args.ik_timeout,
            ik_attempts=args.ik_attempts,
            hand_result_timeout_min_sec=args.hand_result_timeout_min,
            hand_mode=args.hand_mode,
            hand_topic_wait=args.hand_topic_wait,
            arm_send_goal_timeout_sec=args.arm_send_goal_timeout,
            max_joint_jump=args.max_joint_jump,
            skip_hand=args.skip_hand,
            retry_ik_without_collisions=not args.no_ik_collision_fallback,
        )
        node.get_logger().info("Pose sequence complete")
    except Exception as exc:
        node.get_logger().error(f"Pose sequence failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
