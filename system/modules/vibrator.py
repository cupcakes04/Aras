import asyncio
import random
import time
from collections import deque
from .history import History

class VibratorOld(History):
    """Vibrator, functions in 0.0~1.0 signal strength, normalised (Legacy Dummy)"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: float):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.save_history(value)


class Vibrator(History):
    """
    Driver for a single haptic vibrator motor controlled via GPIO.
    Integrated into system.py History structure.

    Parameters: gpio_chip (str), gpio_line (int)
    """

    def __init__(self, gpio_chip: str = '/dev/gpiochip0', gpio_line: int = 10, **kwargs):
        super().__init__(**kwargs)
        self.setup(gpio_chip, gpio_line)
        
    def setup(self, gpio_chip: str, gpio_line: int):
        from periphery import GPIO
        try:
            self._gpio = GPIO(gpio_chip, gpio_line, "out")
            self._gpio.write(False)     # ensure vibrator is off on startup
            self._hardware_available = True
        except Exception as e:
            print(f"[Vibrator {gpio_line}] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False
            
        self._is_on = False

    def close(self):
        """Turn off the vibrator and release GPIO resources."""
        if not self._hardware_available:
            return
        self._gpio.write(False)
        self._gpio.close()

    def _set_hw(self, state: bool):
        if self._hardware_available:
            self._gpio.write(state)
        self._is_on = state

    async def write(self, value: float):
        """
        value: 0.0 ~ 1.0 signal strength.
        Since hardware is binary (on/off), anything > 0 is treated as ON.
        For future PWM implementations, this takes the float.
        """
        state = value > 0.0
        self._set_hw(state)
        self.save_history(value)

