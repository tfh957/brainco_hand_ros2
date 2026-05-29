#!/usr/bin/env python3
import argparse
import math
import sys
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_ACTION = "/rm_group_controller/follow_joint_trajectory"
IK_SERVICE = "/compute_ik"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def rpy_to_quat(roll: float, pitch: float, yaw: float):
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
    return qx, qy, qz, qw


class MoveArmToPose(Node):
    def __init__(self):
        super().__init__("move_arm_to_pose")
        self.ik_client = self.create_client(GetPositionIK, IK_SERVICE)
        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.last_arm_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

    def _on_joint_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in name_to_idx for n in ARM_JOINTS):
            return
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(msg.position[name_to_idx[n]]) for n in ARM_JOINTS]
        self.last_arm_joint_state = js

    def solve_ik(
        self,
        group_name: str,
        ik_link_name: str,
        frame_id: str,
        xyz,
        quat,
        ik_timeout_sec: float,
        ik_attempts: int,
    ) -> JointState:
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/compute_ik service not available")

        last_code = None
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
            req.ik_request.avoid_collisions = True
            if self.last_arm_joint_state is not None:
                req.ik_request.robot_state.joint_state = self.last_arm_joint_state

            fut = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=ik_timeout_sec + 1.0)
            if not fut.done() or fut.result() is None:
                continue
            resp = fut.result()
            if resp.error_code.val == MoveItErrorCodes.SUCCESS:
                return resp.solution.joint_state
            last_code = resp.error_code.val

        if last_code is None:
            raise RuntimeError("IK request timeout")
        raise RuntimeError(f"IK failed, MoveItErrorCode={last_code}")

    def send_arm_goal(self, joint_state: JointState, sec: float):
        if not self.arm_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"{ARM_ACTION} action server not available")

        name_to_idx = {n: i for i, n in enumerate(joint_state.name)}
        if any(n not in name_to_idx for n in ARM_JOINTS):
            raise RuntimeError(f"IK result missing joints: {ARM_JOINTS}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)

        pt = JointTrajectoryPoint()
        pt.positions = [float(joint_state.position[name_to_idx[n]]) for n in ARM_JOINTS]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        goal.trajectory.points = [pt]

        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("send goal timeout")

        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("arm goal rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(10.0, sec + 10.0))
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("arm result timeout")

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"arm action failed, error_code={result.error_code}")


def build_parser():
    p = argparse.ArgumentParser(description="Move arm end-effector to a target pose via MoveIt IK + trajectory action")
    p.add_argument("--group", default="rm65")
    p.add_argument("--ik-link", default="right_base_link")
    p.add_argument("--frame", default="base_link")

    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, required=True)

    p.add_argument("--qx", type=float, default=None)
    p.add_argument("--qy", type=float, default=None)
    p.add_argument("--qz", type=float, default=None)
    p.add_argument("--qw", type=float, default=None)

    p.add_argument("--roll", type=float, default=0.0)
    p.add_argument("--pitch", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)

    p.add_argument("--sec", type=float, default=6.0)
    p.add_argument("--ik-timeout", type=float, default=0.5)
    p.add_argument("--ik-attempts", type=int, default=20)
    return p


def main():
    args = build_parser().parse_args()
    rclpy.init(args=sys.argv)
    node = MoveArmToPose()
    try:
        if None not in (args.qx, args.qy, args.qz, args.qw):
            quat = (args.qx, args.qy, args.qz, args.qw)
        else:
            quat = rpy_to_quat(args.roll, args.pitch, args.yaw)

        joint_state = node.solve_ik(
            group_name=args.group,
            ik_link_name=args.ik_link,
            frame_id=args.frame,
            xyz=(args.x, args.y, args.z),
            quat=quat,
            ik_timeout_sec=args.ik_timeout,
            ik_attempts=args.ik_attempts,
        )
        node.send_arm_goal(joint_state, args.sec)
        node.get_logger().info("Move to pose success")
    except Exception as exc:
        node.get_logger().error(f"Move to pose failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
