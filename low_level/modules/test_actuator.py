"""
test_actuator.py — Linear Actuator Test Script
ARAS (Advanced Rider Assistance System)

Tests the linear actuator by cycling between extended and retracted states.

Usage:
    python3 test_actuator.py

Press Ctrl+C to stop.
"""

import time
from linear_actuator import LinearActuator

actuator = LinearActuator()

print("Linear Actuator Test")
print("Startup — actuator should be extending...")

try:
    while True:
        print("Engaging braking — actuator should retract...")
        actuator.set_braking(True)
        time.sleep(1.8)

        print("Disengaging braking — actuator should extend...")
        actuator.set_braking(False)
        time.sleep(5)

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    actuator.close()