# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse
import math
import time

from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move one actuator to a target angle with a linear ramp."
    )
    parser.add_argument("-c", "--channel", type=str,
                        default="can0", help="CAN channel, e.g. can0")
    parser.add_argument("-i", "--id", type=int,
                        default=1, help="Actuator CAN ID")

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--target-rad", type=float, help="Target angle in radians")
    target_group.add_argument(
        "--target-deg", type=float, help="Target angle in degrees")

    return_group = parser.add_mutually_exclusive_group()
    return_group.add_argument(
        "--return-rad",
        type=float,
        help="Return angle in radians for cyclic tests",
    )
    return_group.add_argument(
        "--return-deg",
        type=float,
        help="Return angle in degrees for cyclic tests",
    )

    parser.add_argument("--kp", type=float, default=0.2,
                        help="Position proportional gain")
    parser.add_argument("--kd", type=float, default=0.005,
                        help="Position derivative gain")
    parser.add_argument("--torque-limit", type=float,
                        default=0.2, help="Torque limit")
    parser.add_argument(
        "--max-speed-deg",
        type=float,
        default=30.0,
        help="Maximum ramp speed in degrees per second",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="How long to hold the target before switching back to idle",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        default=200.0,
        help="Position command update frequency in Hz",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help=(
            "Number of target->return cycles. "
            "If omitted, perform a single move to the target."
        ),
    )
    return parser.parse_args()


def interpolate(start: float, end: float, alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return start * (1.0 - alpha) + end * alpha


def resolve_angle(angle_rad: float | None, angle_deg: float | None, default: float | None = None) -> float | None:
    if angle_rad is not None:
        return angle_rad
    if angle_deg is not None:
        return math.radians(angle_deg)
    return default


def move_to_angle(
    bus: recoil.Bus,
    rate: RateLimiter,
    device_id: int,
    target_angle: float,
    max_speed: float,
    hold_seconds: float,
    stage_name: str,
):
    current_angle = bus.read_position_measured(device_id)
    if current_angle is None:
        raise RuntimeError(f"Failed to read actuator #{device_id} position")

    ramp_duration = max(abs(target_angle - current_angle) / max_speed, 0.2)

    print()
    print(f"[{stage_name}]")
    print(
        f"Current angle: {current_angle:.4f} rad ({math.degrees(current_angle):.2f} deg)")
    print(
        f"Target angle:  {target_angle:.4f} rad ({math.degrees(target_angle):.2f} deg)")
    print(f"Ramp duration: {ramp_duration:.2f} s")

    ramp_start = time.monotonic()
    while True:
        elapsed = time.monotonic() - ramp_start
        alpha = elapsed / ramp_duration
        command_angle = interpolate(current_angle, target_angle, alpha)

        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, command_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(
                f"\rtarget={command_angle:+.4f} rad  "
                f"measured={measured_position:+.4f} rad  "
                f"vel={measured_velocity:+.4f} rad/s",
                end="",
                flush=True,
            )

        if alpha >= 1.0:
            break

        rate.sleep()

    hold_start = time.monotonic()
    while time.monotonic() - hold_start < hold_seconds:
        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, target_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(
                f"\rtarget={target_angle:+.4f} rad  "
                f"measured={measured_position:+.4f} rad  "
                f"vel={measured_velocity:+.4f} rad/s",
                end="",
                flush=True,
            )
        rate.sleep()

    print()


def build_sequence(target_angle: float, return_angle: float | None, cycles: int | None) -> list[tuple[str, float]]:
    if cycles is None:
        return [("target", target_angle)]

    if cycles <= 0:
        raise ValueError("--cycles must be a positive integer")

    if return_angle is None:
        return_angle = 0.0

    sequence = []
    for cycle_index in range(cycles):
        cycle_name = f"cycle {cycle_index + 1}/{cycles}"
        sequence.append((f"{cycle_name}: target", target_angle))
        sequence.append((f"{cycle_name}: return", return_angle))
    return sequence


def main():
    args = parse_args()

    target_angle = resolve_angle(args.target_rad, args.target_deg)
    return_angle = resolve_angle(args.return_rad, args.return_deg)
    if args.cycles is None and return_angle is not None:
        raise ValueError("--return-rad/--return-deg requires --cycles")

    max_speed = math.radians(args.max_speed_deg)
    if max_speed <= 0.0:
        raise ValueError("--max-speed-deg must be positive")

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    rate = RateLimiter(frequency=args.control_rate)
    sequence = build_sequence(target_angle, return_angle, args.cycles)

    print(f"Actuator #{args.id} on {args.channel}")
    print(
        f"Target angle:  {target_angle:.4f} rad ({math.degrees(target_angle):.2f} deg)")
    if args.cycles is not None:
        resolved_return_angle = return_angle if return_angle is not None else 0.0
        print(f"Cycles:        {args.cycles}")
        print(
            f"Return angle:  {resolved_return_angle:.4f} rad "
            f"({math.degrees(resolved_return_angle):.2f} deg)"
        )

    bus.write_position_kp(args.id, args.kp)
    bus.write_position_kd(args.id, args.kd)
    bus.write_torque_limit(args.id, args.torque_limit)
    bus.set_mode(args.id, recoil.Mode.POSITION)
    bus.feed(args.id)

    try:
        for stage_name, stage_target in sequence:
            move_to_angle(
                bus=bus,
                rate=rate,
                device_id=args.id,
                target_angle=stage_target,
                max_speed=max_speed,
                hold_seconds=args.hold_seconds,
                stage_name=stage_name,
            )

    except KeyboardInterrupt:
        print("\nInterrupted, switching actuator back to idle.")

    finally:
        bus.set_mode(args.id, recoil.Mode.IDLE)
        bus.stop()
        print("Actuator set to IDLE.")


if __name__ == "__main__":
    main()
