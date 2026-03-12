"""
haptic.py — Haptic Vibrator Library
ARAS (Advanced Rider Assistance System)

Hardware : Haptic vibrator motor (one per handle, left and right)
Interface: Single GPIO pin via python-periphery GPIO
Platform : Radxa Cubie A7A

Each vibrator is controlled by a single GPIO pin:
  - GPIO low  (False) — vibrator off
  - GPIO high (True)  — vibrator on

Two separate instances of this class are created in main.py, one for
each handle (left and right).
"""

from periphery import GPIO


class Haptic:
    """
    Driver for a single haptic vibrator motor controlled via GPIO.

    Create one instance per vibrator (left handle, right handle).

    Parameters
    ----------
    gpio_chip : str
        GPIO chip path, e.g. '/dev/gpiochip0'
    gpio_line : int
        GPIO line number for the vibrator pin.

    Example
    -------
        left_haptic  = Haptic('/dev/gpiochip0', 10)
        right_haptic = Haptic('/dev/gpiochip0', 11)

        left_haptic.on()
        right_haptic.on()

        left_haptic.off()
        right_haptic.off()

        left_haptic.close()
        right_haptic.close()
    """

    def __init__(self, gpio_chip: str, gpio_line: int):
        self._gpio = GPIO(gpio_chip, gpio_line, "out")
        self._gpio.write(False)     # ensure vibrator is off on startup
        self._is_on = False

    def close(self):
        """Turn off the vibrator and release GPIO resources."""
        self.off()
        self._gpio.close()

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def on(self) -> None:
        """Turn the vibrator on."""
        self._gpio.write(True)
        self._is_on = True

    def off(self) -> None:
        """Turn the vibrator off."""
        self._gpio.write(False)
        self._is_on = False

    @property
    def is_on(self) -> bool:
        """True if the vibrator is currently active, False if off. Read-only."""
        return self._is_on