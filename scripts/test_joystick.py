"""
test_joystick.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Run this script to verify joystick input events.
"""

import time

from berkeley_humanoid_lite_lowlevel.policy.gamepad import Se2Gamepad


controller = Se2Gamepad()
controller.run()

try:
    while True:
        commands = controller.commands
        print(
            f"mode={commands['mode_switch']} "
            f"vx={commands['velocity_x']:+.2f} "
            f"vy={commands['velocity_y']:+.2f} "
            f"vyaw={commands['velocity_yaw']:+.2f}",
            end="\r",
            flush=True,
        )
        time.sleep(0.05)
except KeyboardInterrupt:
    print()
    print("Keyboard interrupt")
finally:
    controller.stop()
