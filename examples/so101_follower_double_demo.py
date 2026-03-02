#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
示例：使用 lerobot 连接 SO‑101 双臂跟随机器人 (so101_follower_double)。

请根据实际硬件修改 PORT 与 ROBOT_ID。
"""

from lerobot.robots.so101_follower_double import (
    SO101FollowerDouble,
    SO101FollowerDoubleConfig,
)

# --------------------------------------------------------------
# 需要自行替换的配置
# --------------------------------------------------------------
PORT = "/dev/tty.usbmodemXXX"   # 使用 `lerobot-find-port` 获取的端口
ROBOT_ID = "my_follower_double"  # 为机器人取一个唯一名称
# --------------------------------------------------------------

# 创建机器人配置
config = SO101FollowerDoubleConfig(
    port=PORT,
    id=ROBOT_ID,
)

# 实例化机器人
robot = SO101FollowerDouble(config)

def main():
    # 1. 连接（不校准，若首次使用请将 calibrate=True）
    print(f"[INFO] 正在连接机器人 (port={PORT}, id={ROBOT_ID})...")
    robot.connect(calibrate=False)

    # 2. （可选）校准
    # 若机器人从未校准过，请取消下面两行注释后运行一次
    # print("[INFO] 正在校准机器人...")
    # robot.calibrate()

    # 3. 示例操作：获取并打印关节位置
    positions = robot.get_observation()
    joint_positions = {k: v for k, v in positions.items() if k.endswith('.pos')}
    print("[INFO] 当前关节位置:", joint_positions)

    # 4. 断开连接
    print("[INFO] 正在断开连接")
    robot.disconnect()

if __name__ == "__main__":
    main()