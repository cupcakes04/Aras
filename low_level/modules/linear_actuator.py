"""
linear_actuator.py — Linear Actuator Library
ARAS (Advanced Rider Assistance System)

Hardware : Linear actuator driven by BTS7960 motor driver
Interface: Two GPIO pins (EXTEND and RETRACT) via python-periphery GPIO
Platform : Radxa Cubie A7A

The BTS7960 is an H-bridge motor driver controlled by two GPIO signals:
  EXTEND (PK3) — drives the motor in the forward direction (extend actuator)
  RETRACT (PK4) — drives the motor in the reverse direction (retract actuator)

Only one pin is HIGH at a time. The inactive pin is held LOW.

States:
  FALSE (braking NOT engaged) — actuator extends to full length (EXTEND HIGH)
  TRUE  (braking engaged)     — actuator retracts to minimum length (RETRACT HIGH)
                                 pulls the brake lever

On startup the actuator is driven to the extended (safe) position.
set_braking() returns immediately — it does not block for travel completion.
"""

from periphery import GPIO

# GPIO chip path
_GPIO_CHIP = "/dev/gpiochip0"

# Pin numbers
_EXTEND_PIN = 323   # PK3 — extend direction
_RETRACT_PIN = 324   # PK4 — retract direction


class LinearActuator:
    """
    Driver for a linear actuator controlled by a BTS7960 H-bridge motor driver.

    Uses two GPIO outputs (EXTEND and RETRACT) to extend or retract the actuator.
    Only one GPIO output is HIGH at a time — the other is held LOW.

    Example
    -------
        actuator = LinearActuator()

        # Emergency braking engaged — retract to pull brake lever
        actuator.set_braking(True)

        # Emergency braking disengaged — extend back to rest position
        actuator.set_braking(False)

        actuator.close()
    """

    def __init__(self):
        self._extend = GPIO(_GPIO_CHIP, _EXTEND_PIN, "out")
        self._retract = GPIO(_GPIO_CHIP, _RETRACT_PIN, "out")

        # Ensure both pins start LOW
        self._extend.write(False)
        self._retract.write(False)

        # Track current braking state
        self._braking = None

        # Drive to safe extended position on startup
        self.set_braking(False)

    def close(self):
        """
        Stop the actuator and release GPIO resources.

        Drives the actuator to the extended (safe) position before releasing,
        ensuring the brake lever is not left in a pulled state.
        """
        self.set_braking(False)
        self._extend.write(False)
        self._retract.write(False)
        self._extend.close()
        self._retract.close()

    # ------------------------------------------------------------------
    # Public method
    # ------------------------------------------------------------------

    def set_braking(self, engage: bool) -> None:
        """
        Engage or disengage emergency braking via the linear actuator.

        Returns immediately — does not block for travel completion.

        Parameters
        ----------
        engage : bool
            True  — engage emergency braking: retract actuator to minimum
                    length, pulling the brake lever.
            False — disengage emergency braking: extend actuator to full
                    length, releasing the brake lever.

        Notes
        -----
        Repeated calls with the same state are ignored to avoid toggling
        the GPIO pins unnecessarily.
        """
        if engage == self._braking:
            return  # Already in requested state, do nothing

        if engage:
            # Retract — RETRACT HIGH, EXTEND LOW
            self._extend.write(False)
            self._retract.write(True)
        else:
            # Extend — EXTEND HIGH, RETRACT LOW
            self._retract.write(False)
            self._extend.write(True)

        self._braking = engage