#!/usr/bin/env python
"""
Minimal demo for connecting to SO101FollowerDouble and printing joint positions.
"""

from lerobot.robots.so101_follower_double import SO101FollowerDouble, SO101FollowerDoubleConfig

# Replace these values for your setup.
PORT = "/dev/f-arms"
ROBOT_ID = "my_double_follower_arm"

JOINT_ORDER = [
    "left_shoulder_pan",
    "left_shoulder_lift",
    "left_elbow_flex",
    "left_wrist_flex",
    "left_wrist_roll",
    "left_gripper",
    "right_shoulder_pan",
    "right_shoulder_lift",
    "right_elbow_flex",
    "right_wrist_flex",
    "right_wrist_roll",
    "right_gripper",
]


def main() -> None:
    config = SO101FollowerDoubleConfig(
        port=PORT,
        id=ROBOT_ID,
    )
    robot = SO101FollowerDouble(config)

    print(f"[INFO] 正在连接机器人 (port={PORT}, id={ROBOT_ID})...")
    robot.connect(calibrate=False)

    try:
        positions = robot.get_observation()
        joint_positions = [float(positions[f"{joint}.pos"]) for joint in JOINT_ORDER]
        formatted = "[" + ",".join(f"{value:.1f}" for value in joint_positions) + "]"
        print("[INFO] 当前关节位置:", formatted)
    finally:
        print("[INFO] 正在断开连接")
        robot.disconnect()


if __name__ == "__main__":
    main()
