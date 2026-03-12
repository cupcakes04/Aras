"""
linear_actuator.py — Linear Actuator Library
ARAS (Advanced Rider Assistance System)

Hardware : Linear actuator driven by BTS7960 motor driver
Interface: Two PWM pins (RPWM and LPWM) via python-periphery PWM
Platform : Radxa Cubie A7A

The BTS7960 is an H-bridge motor driver controlled by two PWM signals:
  RPWM — drives the motor in the forward direction (extend actuator)
  LPWM — drives the motor in the reverse direction (retract actuator)

Only one pin is active at a time. The inactive pin is held at 0 duty cycle.
Duty cycle is always 100% (max) because the linear actuator moves slowly
and full power is needed for reliable lever operation.

States:
  FALSE (braking NOT engaged) — actuator extends to full length (RPWM active)
  TRUE  (braking engaged)     — actuator retracts to minimum length (LPWM active)
                                 pulls the brake lever

On startup the actuator is driven to the extended (safe) position.
set_braking() returns immediately — it does not block for travel completion.
"""

from periphery import PWM


# PWM frequency in Hz — 20 kHz is standard for BTS7960, above audible range
_PWM_FREQUENCY = 20000

# Max duty cycle (1.0 = 100%)
_DUTY_CYCLE_MAX = 1.0
_DUTY_CYCLE_OFF = 0.0


class LinearActuator:
    """
    Driver for a linear actuator controlled by a BTS7960 H-bridge motor driver.

    Uses two PWM outputs (RPWM and LPWM) to extend or retract the actuator.
    Only one PWM output is active at a time — the other is held at zero.
    Duty cycle is fixed at 100% for both directions.

    Parameters
    ----------
    rpwm_chip : int
        PWM chip number for the RPWM pin (extend direction).
        e.g. if the pin is /dev/pwmchip0, chip = 0
    rpwm_channel : int
        PWM channel number for the RPWM pin.
    lpwm_chip : int
        PWM chip number for the LPWM pin (retract direction).
    lpwm_channel : int
        PWM channel number for the LPWM pin.

    Example
    -------
        actuator = LinearActuator(
            rpwm_chip=0, rpwm_channel=0,
            lpwm_chip=0, lpwm_channel=1
        )

        # Emergency braking engaged — retract to pull brake lever
        actuator.set_braking(True)

        # Emergency braking disengaged — extend back to rest position
        actuator.set_braking(False)

        actuator.close()
    """

    def __init__(
        self,
        rpwm_chip: int,
        rpwm_channel: int,
        lpwm_chip: int,
        lpwm_channel: int,
    ):
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

        # Track current braking state
        self._braking = None

        # Drive to safe extended position on startup
        self.set_braking(False)

    def close(self):
        """
        Stop the actuator and release PWM resources.

        Drives the actuator to the extended (safe) position before releasing,
        ensuring the brake lever is not left in a pulled state.
        """
        self.set_braking(False)
        self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._lpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._rpwm.disable()
        self._lpwm.disable()
        self._rpwm.close()
        self._lpwm.close()
    
    def keep_pos(self):

        self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
        self._lpwm.duty_cycle = _DUTY_CYCLE_OFF


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
        Repeated calls with the same state are ignored to avoid resetting
        the motor drive unnecessarily.
        """
        if engage == self._braking:
            return  # Already in requested state, do nothing

        if engage:
            # Retract — LPWM active, RPWM off
            self._rpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._lpwm.duty_cycle = _DUTY_CYCLE_MAX
        else:
            # Extend — RPWM active, LPWM off
            self._lpwm.duty_cycle = _DUTY_CYCLE_OFF
            self._rpwm.duty_cycle = _DUTY_CYCLE_MAX

        self._braking = engage