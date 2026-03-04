#!/usr/bin/env python
"""
Run a simple action-group trajectory on SO101FollowerDouble.

Example (single run):
    python examples/so101_follower_double_action_group.py --port COM5 --robot-id demo

Example (API server on port 8081):
    python examples/so101_follower_double_action_group.py --serve --port COM5 --robot-id demo --api-port 8081

Trigger one action-group run (blocking until done):
    curl -X POST http://127.0.0.1:8081/action-group/run

Check service status:
    curl http://127.0.0.1:8081/action-group/status
"""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse
import uvicorn
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

POSE_HOME = [-8.6,-99.7,98.8,67.8,-17.2,5.4,6.2,-99.3,97.3,64.1,6.8,0.4]
POSE_RAISE = [-8.6,-77.1,55.1,36.8,-17.2,5.6,6.3,-71.2,52.9,33.6,6.7,0.4]
POSE_EXTEND = [-8.1,-8.5,-76.7,3.3,-17.6,63.3,5.7,-4.0,-83.0,-5.4,6.6,52.7]
POSE_CLOSE = [-8.3,-100.0,98.6,77.9,-16.8,3.3,5.8,-99.6,96.8,74.7,7.0,1.0]

raise_right1 = [-11.6,-1.5,-94.4,9.5,-17.1,4.3,5.9,-98.9,96.9,64.0,6.5,1.9]
raise_right2 = [73.5,3.6,-86.3,-2.3,-8.4,4.0,5.7,-98.9,97.0,64.0,6.5,1.9]
raise_right3 = [71.0,3.6,-86.3,-2.2,92.6,4.0,5.8,-98.9,97.0,64.0,6.5,1.9]

raise_right4 = [71.0,4.7,-55.6,-1.5,92.6,4.0,5.7,-98.9,97.0,64.0,6.5,1.9]
raise_right5 =  [71.0,-34.3,-87.6,-2.4,92.6,5.6,5.7,-98.9,97.0,64.0,6.5,1.9]
raise_right6 =  [70.9,30.0,-86.4,-1.6,92.6,4.2,5.7,-98.9,97.0,64.0,6.5,1.9]


raise_right7 = [71.1,-24.4,-87.6,-2.4,92.6,5.6,5.8,-98.9,97.0,64.0,6.5,1.9]
raise_right8 = [71.0,2.5,-87.5,-2.4,92.6,4.5,5.7,-98.9,97.0,64.0,6.5,1.9]
raise_right9 = [70.9,2.5,-86.6,20.0,92.5,100.0,5.8,-98.9,97.0,64.0,6.5,1.9]
raise_right10 = [71.0,2.5,-87.4,0.5,92.5,8.4,5.8,-98.9,97.0,64.0,6.5,1.9]

raise_all1 = [-8.3,-99.8,35.2,46.6,-17.1,4.8,5.9,-99.5,37.4,44.4,6.4,1.6]
raise_all2 = [-15.5,-8.0,-76.7,2.0,-16.7,4.5,6.2,-13.5,-79.5,-2.4,6.5,1.6]
raise_all3 = [80.5,-9.0,-77.6,1.5,-16.7,4.5,-78.9,-13.6,-79.8,-2.5,6.5,1.6]
raise_all4 = [80.4,27.1,-76.7,2.2,-17.3,4.5,-79.1,37.6,-95.9,-1.7,6.4,1.6]
raise_all5 =  [69.8,-28.4,-77.6,1.4,-16.7,4.5,-79.2,-15.4,-96.5,-2.5,6.5,1.6]
raise_all6 =  [69.2,29.8,-85.5,2.1,-17.3,4.5,-79.2,30.3,-95.6,-1.8,6.5,1.6]
raise_all7 =  [69.8,-24.9,-86.3,1.4,-16.7,4.5,-78.7,-15.5,-96.4,-2.5,6.5,1.6]
raise_all8 =  [-10.2,-1.8,-85.5,2.0,-16.7,4.5,6.5,-3.8,-87.0,-2.5,6.6,1.6]

TRAJECTORY = [
    ("home", POSE_HOME, 0.5),
    ("raise_right", raise_right1, 3.0),
    ("raise_right", raise_right2, 1.5),
    ("raise_right", raise_right3, 1.5),
    ("raise_right", raise_right4, 1.5),
    ("raise_right", raise_right5, 1.5),
    ("raise_right", raise_right6, 1.5),
    ("raise_right", raise_right7, 1.0),
    ("raise_right", raise_right8, 1.0),
    ("raise_right", raise_right9, 1.0),
    ("raise_right", raise_right10, 1.0),
    ("raise_right", raise_right9, 1.0),
    ("raise_right", raise_right10, 2.0),
    ("return_home", POSE_HOME, 5.0),
    ("raise_all", raise_all1, 3.0),
    ("raise_all", raise_all2, 2.0),
    ("raise_all", raise_all3, 1.5),
    ("raise_all", raise_all4, 1.5),
    ("raise_all", raise_all5, 1.5),
    ("raise_all", raise_all6, 1.5),
    ("raise_all", raise_all7, 1.5),
    ("raise_all", raise_all8, 1.5),
    ("return_home", POSE_HOME, 5.0),
#    ("raise", POSE_RAISE, 5.0),
#    ("extend", POSE_EXTEND, 8.0),
#    ("return_home", POSE_HOME, 15),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO-101 follower double action-group demo.")
    parser.add_argument("--port", required=True, help="Robot serial port, e.g. COM5 or /dev/ttyUSB0.")
    parser.add_argument("--robot-id", default="my_follower_double", help="Robot identifier string.")
    parser.add_argument("--serve", action="store_true", help="Run FastAPI service instead of one-shot run.")
    parser.add_argument("--host", default="0.0.0.0", help="FastAPI bind host in service mode.")
    parser.add_argument("--api-port", type=int, default=8081, help="FastAPI bind port in service mode.")
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


def run_trajectory(robot: SO101FollowerDouble) -> None:
    total_steps = len(TRAJECTORY)
    for step_index, (name, pose, hold_seconds) in enumerate(TRAJECTORY, start=1):
        if hold_seconds < 0:
            raise ValueError(f"hold_seconds must be >= 0. Got {hold_seconds} for step '{name}'.")
        print(f"[run] step {step_index}/{total_steps}: {name} (hold={hold_seconds:.2f}s)")
        action = pose_to_action(pose)
        robot.send_action(action)
        time.sleep(hold_seconds)
        print_joint_positions(robot)


@dataclass
class ServiceState:
    robot: SO101FollowerDouble
    run_lock: threading.Lock
    last_result: str = "idle"


@contextmanager
def try_run_lock(lock: threading.Lock) -> Iterator[bool]:
    acquired = lock.acquire(blocking=False)
    try:
        yield acquired
    finally:
        if acquired:
            lock.release()


def create_robot(port: str, robot_id: str) -> SO101FollowerDouble:
    return SO101FollowerDouble(
        SO101FollowerDoubleConfig(
            port=port,
            id=robot_id,
        )
    )


def create_app(port: str, robot_id: str) -> FastAPI:
    app = FastAPI(title="SO101 Follower Double Action Group")
    state = ServiceState(
        robot=create_robot(port, robot_id),
        run_lock=threading.Lock(),
    )
    app.state.service = state

    @app.on_event("startup")
    def on_startup() -> None:
        print(f"[connect] port={port}, robot_id={robot_id}")
        state.robot.connect(calibrate=False)
        for motor in state.robot.bus.motors:
            state.robot.bus.write("Acceleration", motor, 50)
            state.robot.bus.write("Maximum_Acceleration", motor, 80)
            state.robot.bus.write("Goal_Velocity", motor, 640)
        state.robot.bus.disable_torque()
        state.last_result = "idle"
        print("[ready] robot connected and torque disabled.")

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        if state.robot.is_connected:
            print("[disconnect] closing robot connection...")
            state.robot.disconnect()

    @app.get("/action-group/status")
    def get_status() -> dict[str, Any]:
        return {
            "running": state.run_lock.locked(),
            "last_result": state.last_result,
        }

    @app.post("/action-group/run")
    def run_action_group() -> JSONResponse:
        with try_run_lock(state.run_lock) as acquired:
            if not acquired:
                return JSONResponse(
                    status_code=409,
                    content={
                        "accepted": False,
                        "status": "busy",
                        "detail": "previous action-group is still running",
                    },
                )

            start = time.perf_counter()
            state.last_result = "running"
            try:
                state.robot.bus.enable_torque()
                run_trajectory(state.robot)
                duration_s = time.perf_counter() - start
                state.last_result = "completed"
                return JSONResponse(
                    status_code=200,
                    content={
                        "accepted": True,
                        "status": "completed",
                        "steps": len(TRAJECTORY),
                        "duration_s": round(duration_s, 2),
                    },
                )
            except Exception as exc:
                state.last_result = "failed"
                return JSONResponse(
                    status_code=500,
                    content={
                        "accepted": True,
                        "status": "failed",
                        "detail": str(exc),
                    },
                )
            finally:
                try:
                    state.robot.bus.disable_torque()
                except Exception as disable_exc:
                    print(f"[warn] failed to disable torque after run: {disable_exc}")

    return app


def run_api_server(args: argparse.Namespace) -> int:
    app = create_app(args.port, args.robot_id)
    uvicorn.run(app, host=args.host, port=args.api_port, workers=1)
    return 0


def main() -> int:
    args = parse_args()

    if args.serve:
        return run_api_server(args)

    robot = create_robot(args.port, args.robot_id)
    
    connected = False
    try:
        print("[connect] " f"port={args.port}, robot_id={args.robot_id}")
        robot.connect(calibrate=False)
        connected = True
        for motor in robot.bus.motors:
            robot.bus.write("Acceleration", motor, 50)
            robot.bus.write("Maximum_Acceleration", motor, 80)
            robot.bus.write("Goal_Velocity", motor, 640)
        run_trajectory(robot)
        print("[done] trajectory complete.")
        return 0
    finally:
        if connected:
            print("[disconnect] closing robot connection...")
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
