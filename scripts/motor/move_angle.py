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
    return parser.parse_args()


def interpolate(start: float, end: float, alpha: float) -> float:
    alpha = min(max(alpha, 0.0), 1.0)
    return start * (1.0 - alpha) + end * alpha


def main():
    args = parse_args()

    target_angle = args.target_rad
    if target_angle is None:
        target_angle = math.radians(args.target_deg)

    max_speed = math.radians(args.max_speed_deg)
    if max_speed <= 0.0:
        raise ValueError("--max-speed-deg must be positive")

    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    rate = RateLimiter(frequency=args.control_rate)

    current_angle = bus.read_position_measured(args.id)
    if current_angle is None:
        bus.stop()
        raise RuntimeError(f"Failed to read actuator #{args.id} position")

    ramp_duration = max(abs(target_angle - current_angle) / max_speed, 0.2)

    print(f"Actuator #{args.id} on {args.channel}")
    print(
        f"Current angle: {current_angle:.4f} rad ({math.degrees(current_angle):.2f} deg)")
    print(
        f"Target angle:  {target_angle:.4f} rad ({math.degrees(target_angle):.2f} deg)")
    print(f"Ramp duration: {ramp_duration:.2f} s")

    bus.write_position_kp(args.id, args.kp)
    bus.write_position_kd(args.id, args.kd)
    bus.write_torque_limit(args.id, args.torque_limit)
    bus.set_mode(args.id, recoil.Mode.POSITION)
    bus.feed(args.id)

    try:
        ramp_start = time.monotonic()
        while True:
            elapsed = time.monotonic() - ramp_start
            alpha = elapsed / ramp_duration
            command_angle = interpolate(current_angle, target_angle, alpha)

            measured_position, measured_velocity = bus.write_read_pdo_2(
                args.id, command_angle, 0.0)
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
        while time.monotonic() - hold_start < args.hold_seconds:
            measured_position, measured_velocity = bus.write_read_pdo_2(
                args.id, target_angle, 0.0)
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

    except KeyboardInterrupt:
        print("\nInterrupted, switching actuator back to idle.")

    finally:
        bus.set_mode(args.id, recoil.Mode.IDLE)
        bus.stop()
        print("Actuator set to IDLE.")


if __name__ == "__main__":
    main()
