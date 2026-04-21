import asyncio
import random
import time
from collections import deque
from .history import History

class ActuatorOld(History):
    """Linear actuator, functions in extend as True/False only (Legacy Dummy)"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: bool):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.save_history(value)


# GPIO chip path
_GPIO_CHIP  = "/dev/gpiochip0"

# Pin numbe
_EXTEND_PIN  = 323   # PK8 — extend direction
_RETRACT_PIN = 324   # PK9 — retract direction


class Actuator(History):
    """
    Driver for a linear actuator controlled by a BTS7960 H-bridge motor driver.

    Uses two GPIO outputs (EXTEND and RETRACT) to extend or retract the actuator.
    Only one GPIO output is HIGH at a time — the other is held LOW.

    Integrated into system.py History structure.

    States:
      False (braking NOT engaged) — actuator extends to full length (EXTEND HIGH)
      True  (braking engaged)     — actuator retracts to minimum length (RETRACT HIGH)
                                    pulls the brake lever
    """

    def __init__(self, gpio_chip = "/dev/gpiochip0", extend_pin=323, retract_pin=324, **kwargs):
        super().__init__(**kwargs)
        self.setup(gpio_chip, extend_pin, retract_pin)

    def setup(self, gpio_chip: str, extend_pin: int , retract_pin: int):
        try:
            from periphery import GPIO
            self._extend  = GPIO(gpio_chip, extend_pin,  "out")
            self._retract = GPIO(gpio_chip, retract_pin, "out")

            # Ensure both pins start LOW
            self._extend.write(False)
            self._retract.write(False)

            self._hardware_available = True
        except Exception as e:
            print(f"[Actuator] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False

        # Track current braking state
        self._braking = None

        # Drive to safe extended position on startup
        if self._hardware_available:
            self._set_braking_hw(False)

    def close(self):
        """Stop the actuator and release GPIO resources."""
        if not self._hardware_available:
            return
        self._set_braking_hw(False)
        self._extend.write(False)
        self._retract.write(False)
        self._extend.close()
        self._retract.close()

    def keep_pos(self):
        """Hold current position by setting both pins LOW."""
        if not self._hardware_available:
            return
        self._extend.write(False)
        self._retract.write(False)

    def _set_braking_hw(self, engage: bool) -> None:
        """
        Internal hardware method to engage or disengage braking.

        Repeated calls with the same state are ignored.
        """
        if engage == self._braking:
            return

        if engage:
            # Retract — RETRACT HIGH, EXTEND LOW
            self._extend.write(False)
            self._retract.write(True)
        else:
            # Extend — EXTEND HIGH, RETRACT LOW
            self._retract.write(False)
            self._extend.write(True)

        self._braking = engage

    async def write(self, value: bool):
        """
        Engage or disengage emergency braking via the linear actuator.

        Parameters
        ----------
        value : bool
            True  — engage emergency braking (retract actuator, pull brake lever)
            False — disengage emergency braking (extend actuator, release lever)
        """
        if self._hardware_available:
            self._set_braking_hw(value)
        self.save_history(value)
