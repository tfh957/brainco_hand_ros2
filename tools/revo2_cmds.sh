#!/usr/bin/env bash

revo2_pub() {
  local p0=${1:-0.0}
  local p1=${2:-0.0}
  local p2=${3:-0.0}
  local p3=${4:-0.0}
  local p4=${5:-0.0}
  local p5=${6:-0.0}
  local sec=${7:-1}
  ros2 topic pub --once /revo2_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['right_thumb_metacarpal_joint','right_thumb_proximal_joint','right_index_proximal_joint','right_middle_proximal_joint','right_ring_proximal_joint','right_pinky_proximal_joint'], points: [{positions: [$p0,$p1,$p2,$p3,$p4,$p5], time_from_start: {sec: $sec}}]}"
}

revo2_open() {
  ros2 topic pub --once /revo2_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['right_thumb_metacarpal_joint','right_thumb_proximal_joint','right_index_proximal_joint','right_middle_proximal_joint','right_ring_proximal_joint','right_pinky_proximal_joint'], points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 1}}]}"
}

revo2_fist() {
  ros2 topic pub --once /revo2_hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['right_thumb_metacarpal_joint','right_thumb_proximal_joint','right_index_proximal_joint','right_middle_proximal_joint','right_ring_proximal_joint','right_pinky_proximal_joint'], points: [{positions: [1.0,1.2,1.2,1.2,1.2,1.2], time_from_start: {sec: 1}}]}"
}
