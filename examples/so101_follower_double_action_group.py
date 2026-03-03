#!/usr/bin/env python
"""
Run a simple action-group trajectory on SO101FollowerDouble.

Example:
    python examples/so101_follower_double_action_group.py --port COM5 --robot-id demo
"""

from __future__ import annotations

import argparse
import time

from lerobot.robots.so101_follower_double import SO101FollowerDouble, SO101FollowerDoubleConfig

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

POSE_HOME = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
POSE_RAISE = [0, 30, -45, 0, 0, 0, 0, 30, -45, 0, 0, 0]
POSE_EXTEND = [45, 0, -30, 0, 0, 10, -45, 0, -30, 0, 0, 10]
POSE_CLOSE = [45, 0, -30, 0, 0, 80, -45, 0, -30, 0, 0, 80]

TRAJECTORY = [
    ("home", POSE_HOME),
    ("raise", POSE_RAISE),
    ("extend", POSE_EXTEND),
    ("close_gripper", POSE_CLOSE),
    ("return_home", POSE_HOME),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO-101 follower double action-group demo.")
    parser.add_argument("--port", required=True, help="Robot serial port, e.g. COM5 or /dev/ttyUSB0.")
    parser.add_argument("--robot-id", default="my_follower_double", help="Robot identifier string.")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=3.0,
        help="Seconds to hold after each action step (default: 3.0).",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=1,
        help="How many times to repeat the full trajectory (default: 1).",
    )
    parser.add_argument(
        "--calibrate-on-connect",
        action="store_true",
        help="Run robot calibration during connect().",
    )
    return parser.parse_args()


def pose_to_action(pose: list[float]) -> dict[str, float]:
    if len(pose) != len(JOINT_ORDER):
        raise ValueError(
            f"Pose length must be {len(JOINT_ORDER)}. "
            f"Got {len(pose)} with pose={pose}."
        )
    return {f"{joint}.pos": value for joint, value in zip(JOINT_ORDER, pose, strict=True)}


def print_joint_positions(robot: SO101FollowerDouble) -> None:
    obs = robot.get_observation()
    values = []
    for joint in JOINT_ORDER:
        key = f"{joint}.pos"
        values.append(f"{joint}={obs[key]:.2f}")
    print("[state] " + ", ".join(values))


def run_trajectory(robot: SO101FollowerDouble, hold_seconds: float, repeats: int) -> None:
    if hold_seconds < 0:
        raise ValueError(f"--hold-seconds must be >= 0. Got {hold_seconds}.")
    if repeats < 1:
        raise ValueError(f"--repeats must be >= 1. Got {repeats}.")

    total_steps = len(TRAJECTORY) * repeats
    step_index = 0
    for cycle in range(1, repeats + 1):
        print(f"[run] cycle {cycle}/{repeats}")
        for name, pose in TRAJECTORY:
            step_index += 1
            print(f"[run] step {step_index}/{total_steps}: {name}")
            action = pose_to_action(pose)
            robot.send_action(action)
            time.sleep(hold_seconds)
            print_joint_positions(robot)


def main() -> int:
    args = parse_args()

    robot = SO101FollowerDouble(
        SO101FollowerDoubleConfig(
            port=args.port,
            id=args.robot_id,
        )
    )

    connected = False
    try:
        print(
            "[connect] "
            f"port={args.port}, robot_id={args.robot_id}, calibrate={args.calibrate_on_connect}"
        )
        robot.connect(calibrate=args.calibrate_on_connect)
        connected = True

        run_trajectory(robot, hold_seconds=args.hold_seconds, repeats=args.repeats)
        print("[done] trajectory complete.")
        return 0
    finally:
        if connected:
            print("[disconnect] closing robot connection...")
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
