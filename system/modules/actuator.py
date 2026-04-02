import asyncio
import random
import time
from collections import deque
from .history import History
from periphery import PWM

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


# PWM frequency in Hz — 20 kHz is standard for BTS7960, above audible range
_PWM_FREQUENCY = 20000

# Max duty cycle (1.0 = 100%)
_DUTY_CYCLE_MAX = 1.0
_DUTY_CYCLE_OFF = 0.0

class Actuator(History):
    """
    Driver for a linear actuator controlled by a BTS7960 H-bridge motor driver.

    Uses two PWM outputs (RPWM and LPWM) to extend or retract the actuator.
    Only one PWM output is active at a time — the other is held at zero.
    Duty cycle is fixed at 100% for both directions.
    
    Integrated into system.py History structure.
    """

    def __init__(self, rpwm_chip: int = 0, rpwm_channel: int = 0, lpwm_chip: int = 0, lpwm_channel: int = 1, **kwargs):
        super().__init__(**kwargs)
        self.setup(rpwm_chip, rpwm_channel, lpwm_chip, lpwm_channel)
        
    def setup(self, rpwm_chip: int, rpwm_channel: int, lpwm_chip: int, lpwm_channel: int):
        try:
            # Initialise RPWM (extend)
            self._rpwm = PWM(rpwm_chip, rpwm_channel)
            self._rpwm.frequency  = _PWM_FREQUENCY
            self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._rpwm.enable()

            # Initialise LPWM (retract)
            self._lpwm = PWM(lpwm_chip, lpwm_channel)
            self._lpwm.frequency  = _PWM_FREQUENCY
            self._lpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._lpwm.enable()
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
        if not self._hardware_available:
            return
        self._set_braking_hw(False)
        self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._lpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._rpwm.disable()
        self._lpwm.disable()
        self._rpwm.close()
        self._lpwm.close()
    
    def keep_pos(self):
        if not self._hardware_available:
            return
        self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._lpwm.duty_cycle = _DUTY_CYCLE_OFF

    def _set_braking_hw(self, engage: bool) -> None:
        if engage == self._braking:
            return

        if engage:
            # Retract — LPWM active, RPWM off
            self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._lpwm.duty_cycle = _DUTY_CYCLE_MAX
        else:
            # Extend — RPWM active, LPWM off
            self._lpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._rpwm.duty_cycle = _DUTY_CYCLE_MAX

        self._braking = engage

    async def write(self, value: bool):
        """
        value: True = engage emergency braking, False = disengage
        """
        if self._hardware_available:
            self._set_braking_hw(value)
        self.save_history(value)

