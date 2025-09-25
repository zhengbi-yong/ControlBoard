#!/usr/bin/env python3
"""Exercise individual joints through the USB CDC command channel.

The control board expects 11-byte frames that start with ``0x7B`` and end with
an XOR checksum.  When bit 7 of the second byte is set the payload is treated as
an on-line command and the remaining bits encode the target joint identifier.
This helper takes care of the framing so that you can focus on the desired
positions.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "pyserial is required to run this script. Install it with 'pip install pyserial'."
    ) from exc

FRAME_HEADER = 0x7B
COMMAND_MIT_SETPOINT = 0x01
COMMAND_HALT = 0x00

JOINT_NAME_TO_ID = {
    "waist_yaw": 0,
    "neck_yaw": 1,
    "neck_pitch": 2,
    "neck_roll": 3,
    "left_shoulder_pitch": 4,
    "left_shoulder_roll": 5,
    "left_elbow": 6,
    "left_wrist": 7,
    "right_shoulder_pitch": 8,
    "right_shoulder_roll": 9,
    "right_elbow": 10,
    "right_wrist": 11,
    "left_hip_pitch": 12,
    "left_hip_yaw": 13,
    "left_hip_roll": 14,
    "left_knee": 15,
    "left_ankle_pitch": 16,
    "left_ankle_roll": 17,
    "left_toe": 18,
    "right_hip_pitch": 19,
    "right_hip_yaw": 20,
    "right_hip_roll": 21,
    "right_knee": 22,
    "right_ankle_pitch": 23,
    "right_ankle_roll": 24,
    "right_toe": 25,
}


def checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def encode_gain(value: float, scale: float) -> int:
    return int(clamp(round(value / scale), 0, 255))


def encode_signed(value: float, scale: float) -> int:
    raw = int(round(value / scale))
    return int(clamp(raw, -32768, 32767)) & 0xFFFF


def build_frame(
    joint_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    ramp: float,
    command: int = COMMAND_MIT_SETPOINT,
) -> bytes:
    frame = bytearray(11)
    frame[0] = FRAME_HEADER
    frame[1] = 0x80 | (joint_id & 0x7F)
    frame[2] = command & 0xFF

    pos_raw = encode_signed(position, 0.001)
    vel_raw = encode_signed(velocity, 0.001)
    kp_raw = encode_gain(kp, 0.5)
    kd_raw = encode_gain(kd, 0.05)
    ramp_raw = encode_gain(ramp, 0.001)

    frame[3] = pos_raw & 0xFF
    frame[4] = (pos_raw >> 8) & 0xFF
    frame[5] = vel_raw & 0xFF
    frame[6] = (vel_raw >> 8) & 0xFF
    frame[7] = kp_raw
    frame[8] = kd_raw
    frame[9] = ramp_raw
    frame[10] = checksum(frame[:10])

    return bytes(frame)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send MIT-style set-points to the control board over USB CDC.")
    parser.add_argument("port", help="Serial port exposed by the control board (for example COM10 or /dev/ttyACM0).")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=2_000_000,
        help="Baud rate of the USB CDC port. Defaults to 2 000 000 baud.",
    )
    parser.add_argument(
        "--joint",
        action="append",
        required=True,
        help="Name of the joint to exercise. Repeat to command multiple joints concurrently.",
    )
    parser.add_argument("--min", dest="min_pos", type=float, default=0.0, help="Lower position bound in radians.")
    parser.add_argument("--max", dest="max_pos", type=float, default=0.5, help="Upper position bound in radians.")
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.0,
        help="Desired velocity in radians per second (used for both directions).",
    )
    parser.add_argument("--kp", type=float, default=20.0, help="Proportional gain used in the MIT frame.")
    parser.add_argument("--kd", type=float, default=0.5, help="Derivative gain used in the MIT frame.")
    parser.add_argument(
        "--ramp",
        type=float,
        default=0.01,
        help="Maximum position step per control tick in radians (body joints only).",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=2.0,
        help="Seconds to wait after sending a command before moving to the next waypoint.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Number of times the joint should move between the min and max positions.",
    )
    return parser.parse_args()


def resolve_joint_ids(names: list[str]) -> list[int]:
    joint_ids: list[int] = []
    for name in names:
        key = name.lower()
        if key not in JOINT_NAME_TO_ID:
            valid = ", ".join(sorted(JOINT_NAME_TO_ID))
            raise SystemExit(f"Unknown joint '{name}'. Valid options are: {valid}")
        joint_ids.append(JOINT_NAME_TO_ID[key])
    return joint_ids


def run_motion(ser: serial.Serial, joint_ids: list[int], args: argparse.Namespace) -> None:
    positions = [args.min_pos, args.max_pos]
    for cycle in range(args.cycles):
        for target in positions:
            for joint_id in joint_ids:
                frame = build_frame(joint_id, target, args.velocity, args.kp, args.kd, args.ramp)
                ser.write(frame)
            ser.flush()
            time.sleep(args.hold)
    # Return the joints to zero before exiting.
    for joint_id in joint_ids:
        frame = build_frame(joint_id, 0.0, 0.0, args.kp, args.kd, args.ramp, command=COMMAND_HALT)
        ser.write(frame)
    ser.flush()


def main() -> int:
    args = parse_args()
    joint_ids = resolve_joint_ids(args.joint)

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.1) as ser:
            time.sleep(0.5)  # Allow the MCU to arm the USB endpoint.
            run_motion(ser, joint_ids, args)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
