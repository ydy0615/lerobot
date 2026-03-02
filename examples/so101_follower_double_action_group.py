#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
动作组示例：让 SO‑101 双机械臂 (so101_follower_double) 按预设轨迹依次运动。

该脚本演示了如何使用 ``SO101FollowerDouble.send_action`` 向机器人发送
一系列关节位置指令（单位：度），并在每个姿态之间保持一定时间。
请根据实际硬件替换 ``PORT`` 与 ``ROBOT_ID``，并确保已完成校准。

运行方式：
    python examples/so101_follower_double_action_group.py
"""

import time
from typing import List

from lerobot.robots.so101_follower_double import (
    SO101FollowerDouble,
    SO101FollowerDoubleConfig,
)

# ----------------------------------------------------------------------
# 1️⃣ 配置（请自行替换端口和机器人 ID）
# ----------------------------------------------------------------------
PORT = "/dev/tty.usbmodemXXX"          # 使用 ``lerobot-find-port`` 获得的端口
ROBOT_ID = "my_follower_double"        # 为机器人取的唯一标识

config = SO101FollowerDoubleConfig(
    port=PORT,
    id=ROBOT_ID,
)

# ----------------------------------------------------------------------
# 2️⃣ 实例化机器人
# ----------------------------------------------------------------------
robot = SO101FollowerDouble(config)

def connect_robot():
    """连接机器人并确保已校准。"""
    print("[INFO] 正在连接机器人...")
    robot.connect(calibrate=False)   # 已校准则直接连接，若需重新校准请设为 True
    print("[INFO] 连接成功")

def disconnect_robot():
    """安全断开连接。"""
    print("[INFO] 正在断开机器人连接...")
    robot.disconnect()
    print("[INFO] 已断开")

# ----------------------------------------------------------------------
# 3️⃣ 关键姿态（单位：度）
#    每个姿态为 12 轴关节角度列表，顺序对应：
#    left_shoulder_pan, left_shoulder_lift, left_elbow_flex,
#    left_wrist_flex, left_wrist_roll, left_gripper,
#    right_shoulder_pan, right_shoulder_lift, right_elbow_flex,
#    right_wrist_flex, right_wrist_roll, right_gripper
# ----------------------------------------------------------------------
POSE_HOME = [0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0]          # 初始居中位姿
POSE_RAISE = [0, 30, -45, 0, 0, 0,   0, 30, -45, 0, 0, 0]   # 双臂抬起
POSE_EXTEND = [45, 0, -30, 0, 0, 10,   -45, 0, -30, 0, 0, 10] # 双臂伸展并轻闭抓手
POSE_CLOSE = [45, 0, -30, 0, 0, 80,   -45, 0, -30, 0, 0, 80] # 抓手全闭
POSE_RETURN = POSE_HOME                                          # 返回居中

# 将姿态列表组合为动作序列
TRAJECTORY: List[List[float]] = [
    POSE_HOME,
    POSE_RAISE,
    POSE_EXTEND,
    POSE_CLOSE,
    POSE_RETURN,
]

# ----------------------------------------------------------------------
# 4️⃣ 发送姿态函数（使用 send_action）
# ----------------------------------------------------------------------
def move_to_pose(pose: List[float], duration: float = 2.0):
    """
    向机器人发送目标关节位置。

    参数:
        pose: 长度为 12 的关节角度列表（度）。
        duration: 运动时长（秒），用于阻塞等待。
    """
    # 将列表转换为符合 ``send_action`` 要求的字典
    action = {
        f"left_shoulder_pan.pos": pose[0],
        f"left_shoulder_lift.pos": pose[1],
        f"left_elbow_flex.pos": pose[2],
        f"left_wrist_flex.pos": pose[3],
        f"left_wrist_roll.pos": pose[4],
        f"left_gripper.pos": pose[5],
        f"right_shoulder_pan.pos": pose[6],
        f"right_shoulder_lift.pos": pose[7],
        f"right_elbow_flex.pos": pose[8],
        f"right_wrist_flex.pos": pose[9],
        f"right_wrist_roll.pos": pose[10],
        f"right_gripper.pos": pose[11],
    }

    # 发送指令，库内部会处理安全限制
    robot.send_action(action)

    # 简单阻塞，确保动作完成后再继续
    time.sleep(duration)

    # 读取并打印当前关节位置（可选调试信息）
    obs = robot.get_observation()
    joint_positions = {k: v for k, v in obs.items() if k.endswith('.pos')}
    print("[INFO] 当前关节位置:", joint_positions)

# ----------------------------------------------------------------------
# 5️⃣ 主流程
# ----------------------------------------------------------------------
def main():
    connect_robot()

    print("[INFO] 开始执行动作组...")
    for idx, pose in enumerate(TRAJECTORY, start=1):
        print(f"[INFO] 执行姿态 {idx}/{len(TRAJECTORY)}")
        move_to_pose(pose, duration=3.0)   # 每个姿态保持约 3 秒

    print("[INFO] 动作组执行完毕")
    disconnect_robot()

if __name__ == "__main__":
    main()