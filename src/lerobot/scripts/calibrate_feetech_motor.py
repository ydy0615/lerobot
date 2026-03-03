# src/lerobot/scripts/calibrate_feetech_motor.py
"""
为单个 Feetech 舵机标定指定 ID（单电机场景下可省略 --motor 参数）。
"""

import argparse
import logging

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

logger = logging.getLogger(__name__)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a single Feetech motor to a given ID (single‑motor use case)."
    )
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port, e.g. COM3 on Windows or /dev/ttyUSB0 on Linux/macOS.",
    )
    # 单电机场景下不需要显式指定 motor 名称，脚本会自动使用唯一的 motor 键
    parser.add_argument(
        "--model",
        type=str,
        required=True,
        help="Feetech motor model (must exist in tables.py, e.g. sts3215, sts3250, sm8512bl, scs0009).",
    )
    parser.add_argument(
        "--new-id",
        type=int,
        required=True,
        help="目标 ID（1‑254），写入舵机的 ID 寄存器。",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # 只创建一个 motor，键名随意（后面不再需要外部指定）
    motor_name = "motor"
    motor_cfg = {
        motor_name: Motor(
            id=0,
            model=args.model,
            norm_mode=MotorNormMode.RANGE_0_100,
        )
    }  # id=0 为占位，后面会被覆盖

    # 初始化总线（不使用校准文件，因为我们只想写入 ID）
    # 先扫描端口获取实际的 motor ID，确保只连接到唯一的电机
    found = FeetechMotorsBus.scan_port(args.port, calibration=None)
    # `scan_port` 返回 {baudrate: [ids...]}, 取所有发现的 ID
    ids = sorted({id_ for ids in found.values() for id_ in ids})
    if len(ids) != 1:
        raise RuntimeError(
            f"期望在端口 {args.port} 上只连接到一个电机，但检测到 IDs={ids}。请检查连接或手动指定 motor ID。"
        )
    actual_id = ids[0]

    motor_cfg = {
        motor_name: Motor(
            id=actual_id,
            model=args.model,
            norm_mode=MotorNormMode.RANGE_0_100,
        )
    }
    bus = FeetechMotorsBus(port=args.port, motors=motor_cfg, calibration=None)
    # 必须先连接才能进行读写操作
    bus.connect()
    # 1️⃣ 写入新的 ID
    bus.write("ID", motor_name, args.new_id)
    logger.info(f"已写入新 ID {args.new_id} 到舵机（模型 {args.model}）")

    # 已写入新 ID，后续如需验证请自行读取

    # 结束后断开连接，确保端口被释放
    bus.disconnect()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()