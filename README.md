# Berkeley Humanoid Lite Low-level Control

This submodule repository contains the low-level control code for the Berkeley Humanoid Lite robot.


## Installation

```bash
sudo apt install net-tools can-utils
```

```bash
uv sync
```

The default environment covers CAN communication, joystick input, IMU access,
and joint calibration.

Install optional extras when needed:

```bash
# RL / policy inference
uv sync --extra rl

# IMU visualization / test utilities
uv sync --extra tools

# Everything above
uv sync --extra rl --extra tools
```

## Getting Started

### Bring up the CAN interface

The low-level computer connects to the joints via CAN.

Run this script to initialize the CAN interface:

```bash
sudo bash ./scripts/start_can_transports.sh
```

### Verify CAN connection

A Python script is provided to verify the CAN connection to all the joints of the robot:

```bash
uv run python ./scripts/check_connection.py
```

### Test a single CAN actuator

For a quick single-motor smoke test:

```bash
uv run python ./scripts/motor/ping.py -c can0 -i 1
uv run python ./scripts/motor/move_angle.py -c can0 -i 1 --target-deg 5
```

To repeat a small motion cycle directly from Python:

```bash
uv run python ./scripts/motor/move_angle.py -c can0 -i 1 --target-deg 5 --cycles 2
```

If the actuator was power-cycled and needs electrical offset calibration:

```bash
uv run python ./scripts/motor/calibrate_electrical_offset.py -c can0 -i 1
```

### Test joystick input

To verify that the gamepad events are being read correctly:

```bash
uv run python ./scripts/test_joystick.py
```

### Joint Calibration

Because the joint actuators only have single encoder on the motor shaft, we need to calibrate the zero position of the joints after each power cycle.

Run the following command to start the calibration:

```bash
uv run python ./scripts/calibrate_joints.py
```

After the script is launched and running, manually move the robot joints to the mechanical position limits. After all the joints are moved, press `q` or the `B` button on the joystick to quit the calibration.

The calibration data will be saved in the `./calibration.yaml` file.



### Run main controller

The main controller is implemented in C.

To run the controller, run the following command:

```bash
make run
```

Press `LB` + `A` to enter RL init mode. Then, press `RB` + `A` to enter RL running mode.

At any time, press `B` or the thumb buttons to exit RL mode. The joints will enter passive damping mode.

Press `Ctrl` + `C` to terminate the controller. Upon first termination, the joints will enter passive damping mode. Press `Ctrl` + `C` again to completely stop the controller, which joints will return to unpowered idle state.
