# src/lerobot/scripts/calibrate_feetech_motor.py
"""
为单个 Feetech 舵机标定指定 ID（单电机场景下可省略 --motor 参数）。
"""

import argparse
import logging

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorCalibration

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
    motor_cfg = {motor_name: Motor(id=0, model=args.model)}  # id=0 为占位，后面会被覆盖

    # 初始化总线（不使用校准文件，因为我们只想写入 ID）
    bus = FeetechMotorsBus(port=args.port, motors=motor_cfg, calibration=None)

    # 1️⃣ 写入新的 ID
    bus.write("ID", motor_name, args.new_id)
    logger.info(f"已写入新 ID {args.new_id} 到舵机（模型 {args.model}）")

    # 2️⃣ 读取当前校准（如果已有则保留范围/偏移，否则读取现场范围）
    current_cal = bus.read_calibration()
    if motor_name in current_cal:
        cal = current_cal[motor_name]
    else:
        # 读取位置限位作为默认范围
        min_pos = bus.read("Min_Position_Limit", motor_name, normalize=False)
        max_pos = bus.read("Max_Position_Limit", motor_name, normalize=False)
        cal = MotorCalibration(
            id=args.new_id,
            drive_mode=0,
            homing_offset=0,
            range_min=min_pos,
            range_max=max_pos,
        )
    cal.id = args.new_id  # 更新 ID

    # 3️⃣ 写回校准信息并缓存到磁盘（cache=True 会自动生成 .json 文件）
    bus.write_calibration({motor_name: cal}, cache=True)
    logger.info(f"校准文件已更新，ID 为 {cal.id}")

    # 4️⃣ 可选验证：读取 ID 确认写入成功
    read_id = bus.read("ID", motor_name, normalize=False)
    if read_id == args.new_id:
        logger.info("验证成功：舵机 ID 已正确写入。")
    else:
        logger.warning(
            f"验证失败：读取的 ID 为 {read_id}，预期为 {args.new_id}。请检查串口和电源。"
        )

    # 5️⃣ 恢复扭矩状态（防止舵机保持锁定）
    bus.enable_torque([motor_name])
    bus.disable_torque([motor_name])


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()