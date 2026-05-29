#!/usr/bin/env python3
import dataclasses
import sys
from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_ACTION = "/rm_group_controller/follow_joint_trajectory"
HAND_ACTION = "/revo2_hand_controller/follow_joint_trajectory"

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HAND_JOINTS = [
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]


@dataclasses.dataclass
class Step:
    name: str
    arm: List[float]
    hand: List[float]
    arm_sec: float
    hand_sec: float


class SequenceDemo(Node):
    def __init__(self):
        super().__init__("rm_revo2_sequence_demo")
        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.hand_client = ActionClient(self, FollowJointTrajectory, HAND_ACTION)
        self.result_timeout_sec = 20.0

    def _goal(self, joint_names: List[str], positions: List[float], sec: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        goal.trajectory.points = [pt]
        return goal

    def _send_and_wait(self, client: ActionClient, goal: FollowJointTrajectory.Goal, label: str):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{label} server unavailable")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError(f"{label} send goal timeout")

        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError(f"{label} goal rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.result_timeout_sec)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError(f"{label} timeout waiting result ({self.result_timeout_sec}s)")

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"{label} failed, error_code={result.error_code}")
        self.get_logger().info(f"{label} success")

    def run(self, steps: List[Step]):
        for i, step in enumerate(steps, 1):
            self.get_logger().info(f"[Step {i}] {step.name}")
            arm_goal = self._goal(ARM_JOINTS, step.arm, step.arm_sec)
            self._send_and_wait(self.arm_client, arm_goal, f"{step.name} arm")

            hand_goal = self._goal(HAND_JOINTS, step.hand, step.hand_sec)
            self._send_and_wait(self.hand_client, hand_goal, f"{step.name} hand")


def default_steps():
    return [
        Step(
            name="arm_to_pregrasp",
            arm=[0.0, -0.2, 0.3, 0.0, 0.2, 0.0],
            hand=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            arm_sec=2.0,
            hand_sec=1.0,
        ),
        Step(
            name="arm_to_pick",
            arm=[0.05, 0.15, 0.25, 0.0, 0.25, 0.0],
            hand=[1.0, 1.2, 1.2, 1.2, 1.2, 1.2],
            arm_sec=2.0,
            hand_sec=1.0,
        ),
    ]


def main():
    rclpy.init(args=sys.argv)
    node = SequenceDemo()
    try:
        node.run(default_steps())
    except Exception as exc:
        node.get_logger().error(f"Sequence failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
