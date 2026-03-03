# src/lerobot/scripts/calibrate_feetech_motor.py
"""
为单个 Feetech 舵机标定指定 ID（单电机场景下可省略 --motor 参数）。
"""

import argparse
import logging
import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.motors_bus import get_address

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

    motor_name = "motor"
    motor_cfg = {
        motor_name: Motor(
            id=0,                     # 占位 ID，实际写入时会被 new-id 替换
            model=args.model,
            norm_mode=MotorNormMode.RANGE_0_100,
        )
    }

    bus = FeetechMotorsBus(port=args.port, motors=motor_cfg, calibration=None)
    bus.connect()
    # 关闭扭矩以允许写入 EEPROM
    bus.disable_torque([motor_name])
    # 使用 Reg Write（持久化写入）写入 ID
    addr, length = get_address(bus.model_ctrl_table, args.model, "ID")
    data = bus._serialize_data(args.new_id, length)
    bus.packet_handler.regWriteTxOnly(bus.port_handler, bus.motors[motor_name].id, addr, length, data)
    # 提交写入
    bus.packet_handler.action(bus.port_handler, bus.motors[motor_name].id)
    # 等待 EEPROM 写入完成（约 0.7 秒）
    time.sleep(0.7)
    logger.info(f"已永久写入新 ID {args.new_id}（模型 {args.model}）")
    bus.disconnect()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()