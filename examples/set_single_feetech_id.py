#!/usr/bin/env python
"""
Set the ID of a single connected Feetech motor (STS3215 by default) permanently.

This script uses `FeetechMotorsBus.setup_motor()` to write the target ID in EPROM
and then verifies the motor responds on the default bus baudrate (1,000,000).

Example:
    python examples/set_single_feetech_id.py --port COM5 --target-id 3
"""

from __future__ import annotations

import argparse
import sys
from typing import Tuple

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, MODEL_PROTOCOL


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Permanently set ID for a single connected Feetech servo."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument(
        "--target-id",
        type=int,
        required=True,
        help="Target motor ID to write permanently (must not be broadcast ID).",
    )
    parser.add_argument(
        "--model",
        default="sts3215",
        help="Motor model key supported by lerobot (default: sts3215).",
    )
    parser.add_argument(
        "--motor-name",
        default="single_motor",
        help="Internal motor name used by this script (default: single_motor).",
    )
    parser.add_argument(
        "--initial-id",
        type=int,
        default=None,
        help="Known current ID to skip discovery scan.",
    )
    parser.add_argument(
        "--initial-baudrate",
        type=int,
        default=None,
        help="Known current baudrate to skip discovery scan.",
    )
    return parser.parse_args()


def _import_scservo_sdk():
    try:
        import scservo_sdk as scs
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing dependency 'scservo_sdk'. Install Feetech support first: "
            "pip install -e \".[feetech]\""
        ) from exc
    return scs


def validate_args(args: argparse.Namespace, scs) -> None:
    if args.target_id < 0 or args.target_id >= scs.BROADCAST_ID:
        raise ValueError(
            f"--target-id must satisfy 0 <= id < {scs.BROADCAST_ID} (broadcast ID is not allowed). "
            f"Got {args.target_id}."
        )

    if args.model not in FeetechMotorsBus.model_number_table:
        supported = ", ".join(sorted(FeetechMotorsBus.model_number_table))
        raise ValueError(f"Unsupported --model '{args.model}'. Supported models: {supported}")

    model_protocol = MODEL_PROTOCOL.get(args.model)
    if model_protocol != 0:
        raise ValueError(
            f"This script is intended for protocol 0 motors (STS/SMS series). "
            f"Model '{args.model}' uses protocol {model_protocol}."
        )

    if args.initial_id is not None and (args.initial_id < 0 or args.initial_id >= scs.BROADCAST_ID):
        raise ValueError(
            f"--initial-id must satisfy 0 <= id < {scs.BROADCAST_ID}. Got {args.initial_id}."
        )

    if args.initial_baudrate is not None and args.initial_baudrate <= 0:
        raise ValueError(f"--initial-baudrate must be > 0. Got {args.initial_baudrate}.")


def discover_single_motor(port: str) -> Tuple[int, int]:
    print("[precheck] Scanning port to ensure exactly one motor is connected...")
    baudrate_to_ids = FeetechMotorsBus.scan_port(port, protocol_version=0)
    if not baudrate_to_ids:
        raise RuntimeError(
            "No motor detected. Check power supply, 3-pin cable, USB cable, and --port."
        )

    found_pairs: list[tuple[int, int]] = []
    for baudrate, ids in baudrate_to_ids.items():
        for motor_id in ids:
            found_pairs.append((baudrate, motor_id))

    if len(found_pairs) != 1:
        discovered = ", ".join([f"(baudrate={b}, id={i})" for b, i in found_pairs])
        raise RuntimeError(
            "Expected exactly one connected motor, but detected multiple candidates: "
            f"{discovered}. Connect only one motor and retry."
        )

    baudrate, motor_id = found_pairs[0]
    print(f"[precheck] Found one motor at baudrate={baudrate}, id={motor_id}.")
    return baudrate, motor_id


def build_bus(args: argparse.Namespace) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=args.port,
        motors={args.motor_name: Motor(args.target_id, args.model, MotorNormMode.DEGREES)},
        protocol_version=0,
    )


def safe_disconnect(bus: FeetechMotorsBus | None) -> None:
    if bus is None:
        return

    try:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)
            return
    except Exception:
        pass

    # Best-effort close if the bus is in a partially connected state.
    try:
        bus.port_handler.closePort()
    except Exception:
        pass


def print_troubleshooting() -> None:
    print(
        "\nTroubleshooting:\n"
        "1) Ensure only one motor is connected to the controller board.\n"
        "2) Ensure external motor power is on and stable.\n"
        "3) Verify --port is correct (use `lerobot-find-port`).\n"
        "4) Ensure Feetech dependency is installed: pip install -e \".[feetech]\"."
    )


def main() -> int:
    args = parse_args()
    bus: FeetechMotorsBus | None = None

    try:
        scs = _import_scservo_sdk()
        validate_args(args, scs)

        initial_baudrate = args.initial_baudrate
        initial_id = args.initial_id

        # Auto-discover only when either field is missing.
        if initial_baudrate is None or initial_id is None:
            discovered_baudrate, discovered_id = discover_single_motor(args.port)
            if initial_baudrate is None:
                initial_baudrate = discovered_baudrate
            if initial_id is None:
                initial_id = discovered_id

        bus = build_bus(args)

        print(
            "[write] Writing ID in motor EPROM via setup_motor(): "
            f"model={args.model}, target_id={args.target_id}"
        )
        bus.setup_motor(args.motor_name, initial_baudrate=initial_baudrate, initial_id=initial_id)

        print(f"[verify] Switching bus to default baudrate={bus.default_baudrate} and pinging target ID...")
        bus.set_baudrate(bus.default_baudrate)
        expected_model_nb = bus.model_number_table[args.model]
        found_model_nb = bus.ping(args.target_id, raise_on_error=True)
        if found_model_nb is None:
            raise RuntimeError(
                f"Ping returned no response for target ID {args.target_id} at baudrate {bus.default_baudrate}."
            )
        if found_model_nb != expected_model_nb:
            raise RuntimeError(
                f"Model mismatch after write: expected model number {expected_model_nb}, "
                f"but got {found_model_nb} on ID {args.target_id}."
            )

        print(
            f"[ok] ID has been permanently written to {args.target_id}. "
            f"Motor now responds at baudrate {bus.default_baudrate}."
        )
        print(
            "[next] To verify persistence, power-cycle the motor and run this script again "
            f"with --target-id {args.target_id}."
        )
        return 0

    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        print_troubleshooting()
        return 1
    finally:
        safe_disconnect(bus)


if __name__ == "__main__":
    raise SystemExit(main())
