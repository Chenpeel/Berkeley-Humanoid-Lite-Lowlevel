# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from __future__ import annotations

import argparse
import errno
import math
import time

import berkeley_humanoid_lite_lowlevel.recoil as recoil

DEFAULT_ACTUATOR_BITRATE = 1_000_000
PARAMETER_APPLY_DELAY_SECONDS = 0.001


class SimpleRateLimiter:
    def __init__(self, frequency_hz: float) -> None:
        if frequency_hz <= 0.0:
            raise ValueError("control_frequency_hz must be positive")

        self._period_seconds = 1.0 / frequency_hz
        self._next_tick_time: float | None = None

    def sleep(self) -> None:
        now = time.monotonic()
        if self._next_tick_time is None:
            self._next_tick_time = now + self._period_seconds
            return

        remaining_seconds = self._next_tick_time - now
        if remaining_seconds > 0.0:
            time.sleep(remaining_seconds)
            self._next_tick_time += self._period_seconds
            return

        self._next_tick_time = now + self._period_seconds


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move one actuator to a target angle with a linear ramp."
    )
    parser.add_argument(
        "-c",
        "--channel",
        type=str,
        default="can0",
        help="CAN transport channel",
    )
    parser.add_argument(
        "-i",
        "--id",
        type=int,
        default=1,
        help="Actuator device identifier",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=DEFAULT_ACTUATOR_BITRATE,
        help="CAN transport bitrate",
    )

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-rad", type=float, help="Target angle in radians")
    target_group.add_argument("--target-deg", type=float, help="Target angle in degrees")

    return_group = parser.add_mutually_exclusive_group()
    return_group.add_argument("--return-rad", type=float, help="Return angle in radians for cyclic tests")
    return_group.add_argument("--return-deg", type=float, help="Return angle in degrees for cyclic tests")

    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        help="Optional position proportional gain override. Omit to keep the device value.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        help="Optional position derivative gain override. Omit to keep the device value.",
    )
    parser.add_argument(
        "--torque-limit",
        type=float,
        default=None,
        help="Optional torque limit override. Omit to keep the device value.",
    )
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
        help="How long to hold the target before moving to the next stage",
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
            "Number of target to return cycles. "
            "If omitted, perform a single move to the target."
        ),
    )
    return parser.parse_args()


def resolve_angle_radians(
    angle_radians: float | None,
    angle_degrees: float | None,
    *,
    default: float | None = None,
) -> float | None:
    if angle_radians is not None:
        return angle_radians
    if angle_degrees is not None:
        return math.radians(angle_degrees)
    return default


def interpolate_value(start: float, end: float, alpha: float) -> float:
    clamped_alpha = min(max(alpha, 0.0), 1.0)
    return start * (1.0 - clamped_alpha) + end * clamped_alpha


def build_angle_sequence(
    target_angle_radians: float,
    return_angle_radians: float | None = None,
    cycles: int | None = None,
) -> list[tuple[str, float]]:
    if cycles is None:
        return [("target", target_angle_radians)]

    if cycles <= 0:
        raise ValueError("--cycles must be a positive integer")

    resolved_return_angle = 0.0 if return_angle_radians is None else return_angle_radians
    sequence: list[tuple[str, float]] = []
    for cycle_index in range(cycles):
        cycle_name = f"cycle {cycle_index + 1}/{cycles}"
        sequence.append((f"{cycle_name}: target", target_angle_radians))
        sequence.append((f"{cycle_name}: return", resolved_return_angle))
    return sequence


def print_motion_status(
    *,
    command_angle_radians: float,
    measured_position: float | None,
    measured_velocity: float | None,
) -> None:
    if measured_position is None or measured_velocity is None:
        return

    print(
        f"\rtarget={command_angle_radians:+.4f} rad  "
        f"measured={measured_position:+.4f} rad  "
        f"vel={measured_velocity:+.4f} rad/s",
        end="",
        flush=True,
    )


def enter_position_mode(
    bus: recoil.Bus,
    device_id: int,
    *,
    position_kp: float | None,
    position_kd: float | None,
    torque_limit: float | None,
) -> None:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    time.sleep(PARAMETER_APPLY_DELAY_SECONDS)

    if position_kp is not None:
        bus.write_position_kp(device_id, position_kp)
        time.sleep(PARAMETER_APPLY_DELAY_SECONDS)

    if position_kd is not None:
        bus.write_position_kd(device_id, position_kd)
        time.sleep(PARAMETER_APPLY_DELAY_SECONDS)

    if torque_limit is not None:
        bus.write_torque_limit(device_id, torque_limit)
        time.sleep(PARAMETER_APPLY_DELAY_SECONDS)

    bus.feed(device_id)
    bus.set_mode(device_id, recoil.Mode.POSITION)


def move_to_angle(
    bus: recoil.Bus,
    rate: SimpleRateLimiter,
    device_id: int,
    *,
    target_angle_radians: float,
    max_speed_radians_per_second: float,
    hold_seconds: float,
    stage_name: str,
) -> None:
    current_angle_radians = bus.read_position_measured(device_id)
    if current_angle_radians is None:
        raise RuntimeError(f"Failed to read actuator #{device_id} position")

    ramp_duration_seconds = max(
        abs(target_angle_radians - current_angle_radians) / max_speed_radians_per_second,
        0.2,
    )

    print()
    print(f"[{stage_name}]")
    print(
        f"Current angle: {current_angle_radians:.4f} rad "
        f"({math.degrees(current_angle_radians):.2f} deg)"
    )
    print(
        f"Target angle:  {target_angle_radians:.4f} rad "
        f"({math.degrees(target_angle_radians):.2f} deg)"
    )
    print(f"Ramp duration: {ramp_duration_seconds:.2f} s")

    ramp_start_time = time.monotonic()
    while True:
        elapsed_seconds = time.monotonic() - ramp_start_time
        alpha = elapsed_seconds / ramp_duration_seconds
        command_angle_radians = interpolate_value(current_angle_radians, target_angle_radians, alpha)

        bus.feed(device_id)
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, command_angle_radians, 0.0)
        print_motion_status(
            command_angle_radians=command_angle_radians,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
        )

        if alpha >= 1.0:
            break

        rate.sleep()

    hold_start_time = time.monotonic()
    while time.monotonic() - hold_start_time < hold_seconds:
        bus.feed(device_id)
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle_radians, 0.0)
        print_motion_status(
            command_angle_radians=target_angle_radians,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
        )
        rate.sleep()

    print()


def run_angle_sequence(
    *,
    bus: recoil.Bus,
    device_id: int,
    target_angle_radians: float,
    return_angle_radians: float | None,
    cycles: int | None,
    position_kp: float | None,
    position_kd: float | None,
    torque_limit: float | None,
    max_speed_radians_per_second: float,
    hold_seconds: float,
    control_frequency_hz: float,
) -> None:
    rate = SimpleRateLimiter(control_frequency_hz)
    sequence = build_angle_sequence(
        target_angle_radians=target_angle_radians,
        return_angle_radians=return_angle_radians,
        cycles=cycles,
    )

    enter_position_mode(
        bus,
        device_id,
        position_kp=position_kp,
        position_kd=position_kd,
        torque_limit=torque_limit,
    )

    try:
        for stage_name, stage_target_radians in sequence:
            move_to_angle(
                bus,
                rate,
                device_id,
                target_angle_radians=stage_target_radians,
                max_speed_radians_per_second=max_speed_radians_per_second,
                hold_seconds=hold_seconds,
                stage_name=stage_name,
            )
    except KeyboardInterrupt:
        print("\nInterrupted, switching actuator back to idle.")
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)


def main() -> None:
    args = parse_args()

    target_angle_radians = resolve_angle_radians(args.target_rad, args.target_deg)
    if target_angle_radians is None:
        raise ValueError("A target angle must be provided")

    return_angle_radians = resolve_angle_radians(args.return_rad, args.return_deg)
    if args.cycles is None and return_angle_radians is not None:
        raise ValueError("--return-rad/--return-deg requires --cycles")

    if args.max_speed_deg <= 0.0:
        raise ValueError("--max-speed-deg must be positive")
    if args.hold_seconds < 0.0:
        raise ValueError("--hold-seconds must be non-negative")
    if args.control_rate <= 0.0:
        raise ValueError("--control-rate must be positive")

    max_speed_radians_per_second = math.radians(args.max_speed_deg)
    resolved_return_angle = 0.0 if return_angle_radians is None else return_angle_radians

    print(f"Actuator #{args.id} on {args.channel}")
    print(
        f"Target angle:  {target_angle_radians:.4f} rad "
        f"({math.degrees(target_angle_radians):.2f} deg)"
    )
    if args.cycles is not None:
        print(f"Cycles:        {args.cycles}")
        print(
            f"Return angle:  {resolved_return_angle:.4f} rad "
            f"({math.degrees(resolved_return_angle):.2f} deg)"
        )
    if args.kp is not None:
        print(f"Kp override:   {args.kp:.4f}")
    if args.kd is not None:
        print(f"Kd override:   {args.kd:.4f}")
    if args.torque_limit is not None:
        print(f"Torque limit:  {args.torque_limit:.4f}")

    bus = recoil.Bus(channel=args.channel, bitrate=args.bitrate)
    try:
        run_angle_sequence(
            bus=bus,
            device_id=args.id,
            target_angle_radians=target_angle_radians,
            return_angle_radians=return_angle_radians,
            cycles=args.cycles,
            position_kp=args.kp,
            position_kd=args.kd,
            torque_limit=args.torque_limit,
            max_speed_radians_per_second=max_speed_radians_per_second,
            hold_seconds=args.hold_seconds,
            control_frequency_hz=args.control_rate,
        )
    finally:
        bus.stop()


def run_with_friendly_errors() -> None:
    try:
        main()
    except ModuleNotFoundError as error:
        if error.name == "can":
            raise SystemExit(
                "Missing dependency `python-can`. "
                "Install it first, then retry."
            ) from error
        raise
    except OSError as error:
        if error.errno != errno.ENODEV:
            raise

        raise SystemExit(
            "No CAN device is available. "
            "Bring up the target CAN interface first, then retry."
        ) from error


if __name__ == "__main__":
    run_with_friendly_errors()
