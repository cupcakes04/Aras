#!/usr/bin/env python3
"""
BTS7960 motor driver control (Python / Linux) using python-periphery.

Arduino original behavior:
- Enable both sides
- Ramp CW:  R_PWM increases, L_PWM = 0
- Ramp CCW: L_PWM increases, R_PWM = 0
"""

import time
from periphery import GPIO, PWM


# ----------------------------
# 1) EDIT THESE MAPPINGS
# ----------------------------
# GPIO mapping (Linux gpiochip device + line offset)
# Find these with:
#   ls /dev/gpiochip*
#   gpioinfo            (from package: gpiod)
#
# Example format:
#   ("gpiochip2", 17)  -> /dev/gpiochip2 line 17
R_IS_CHIP, R_IS_LINE = "gpiochipX", 0    # <-- EDIT
R_EN_CHIP, R_EN_LINE = "gpiochipX", 0    # <-- EDIT
L_IS_CHIP, L_IS_LINE = "gpiochipX", 0    # <-- EDIT
L_EN_CHIP, L_EN_LINE = "gpiochipX", 0    # <-- EDIT

# PWM mapping (pwmchip name + channel)
# Find these with:
#   ls /sys/class/pwm
#   ls /sys/class/pwm/pwmchip*/       (see how many channels exist: npwm)
#
# Example format:
#   ("pwmchip0", 0) -> /sys/class/pwm/pwmchip0 channel 0
R_PWM_CHIP, R_PWM_CH = "pwmchipX", 0      # <-- EDIT
L_PWM_CHIP, L_PWM_CH = "pwmchipX", 0      # <-- EDIT


# ----------------------------
# 2) TUNABLE SETTINGS
# ----------------------------
PWM_FREQ_HZ = 20000        # 20 kHz is common for DC motor drivers (quiet)
STEP = 10                  # Arduino increments by 10
STEP_DELAY_S = 0.5         # Arduino delay(500)
PAUSE_S = 0.5              # Arduino delay(500) between ramps


def duty_from_0_255(val: int) -> float:
    """Convert Arduino analogWrite range [0..255] to duty cycle [0.0..1.0]."""
    val = max(0, min(255, val))
    return val / 255.0


def pwm_setup(chip: str, channel: int, freq_hz: int) -> PWM:
    pwm = PWM(chip, channel)

    # python-periphery PWM uses:
    # - period_ns
    # - duty_cycle_ns  (or duty_cycle depending on version)
    period_ns = int(1e9 / freq_hz)

    pwm.period = period_ns
    pwm.duty_cycle = 0.0  # fraction 0.0..1.0 supported in newer periphery
    pwm.enable()
    return pwm


def main():
    # GPIOs
    r_is = GPIO(R_IS_CHIP, R_IS_LINE, "out")
    r_en = GPIO(R_EN_CHIP, R_EN_LINE, "out")
    l_is = GPIO(L_IS_CHIP, L_IS_LINE, "out")
    l_en = GPIO(L_EN_CHIP, L_EN_LINE, "out")

    # PWM
    r_pwm = pwm_setup(R_PWM_CHIP, R_PWM_CH, PWM_FREQ_HZ)
    l_pwm = pwm_setup(L_PWM_CHIP, L_PWM_CH, PWM_FREQ_HZ)

    try:
        # Match Arduino setup()
        r_is.write(False)   # LOW
        l_is.write(False)   # LOW
        r_en.write(True)    # HIGH
        l_en.write(True)    # HIGH

        while True:
            # Clockwise: R ramps up, L = 0
            for i in range(0, 256, STEP):
                r_pwm.duty_cycle = duty_from_0_255(i)
                l_pwm.duty_cycle = 0.0
                time.sleep(STEP_DELAY_S)

            time.sleep(PAUSE_S)

            # Counter-clockwise: L ramps up, R = 0
            for i in range(0, 256, STEP):
                r_pwm.duty_cycle = 0.0
                l_pwm.duty_cycle = duty_from_0_255(i)
                time.sleep(STEP_DELAY_S)

            time.sleep(PAUSE_S)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop motor + cleanup
        try:
            r_pwm.duty_cycle = 0.0
            l_pwm.duty_cycle = 0.0
        except Exception:
            pass

        for obj in (r_pwm, l_pwm, r_is, r_en, l_is, l_en):
            try:
                obj.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
