import time

def test_pwm():
    from linear_actuator import LinearActuator
    # Replace these with your actual PWM chip and channel numbers
    actuator = LinearActuator(
        rpwm_chip=10, rpwm_channel=2,
        lpwm_chip=10, lpwm_channel=3
    )

    try:
        while True:
            print("Engaging braking — actuator should retract...")
            actuator.set_braking(True)
            time.sleep(1.67) # give it time to fully retract

            actuator.keep_pos()
            time.sleep(5)

            print("Disengaging braking — actuator should extend...")
            actuator.set_braking(False)
            time.sleep(5)  # give it time to fully extend
    except KeyboardInterrupt:
        print("Done.")
    finally:
        actuator.close()


def test_digital():
    from periphery import GPIO
    
    # Replace with your actual GPIO pin numbers corresponding to RPWM and LPWM
    RPWM_PIN = 100 
    LPWM_PIN = 101

    print(f"Initializing GPIO pins: RPWM={RPWM_PIN}, LPWM={LPWM_PIN}")
    rpwm = GPIO(RPWM_PIN, "out")
    lpwm = GPIO(LPWM_PIN, "out")

    try:
        while True:
            print("Engaging braking (Digital) — LPWM HIGH, RPWM LOW (retracting)")
            rpwm.write(False)
            lpwm.write(True)
            time.sleep(1.67)

            print("Keeping position (Digital) — LPWM LOW, RPWM LOW")
            rpwm.write(False)
            lpwm.write(False)
            time.sleep(5)

            print("Disengaging braking (Digital) — LPWM LOW, RPWM HIGH (extending)")
            lpwm.write(False)
            rpwm.write(True)
            time.sleep(5)
    except KeyboardInterrupt:
        print("Done.")
    finally:
        rpwm.write(False)
        lpwm.write(False)
        rpwm.close()
        lpwm.close()

if __name__ == "__main__":
    # Choose which test to run
    # test_pwm()
    test_digital()