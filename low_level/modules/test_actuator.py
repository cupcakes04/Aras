import time
from linear_actuator import LinearActuator

# Replace these with your actual PWM chip and channel numbers
actuator = LinearActuator(
    rpwm_chip=10, rpwm_channel=2,
    lpwm_chip=10, lpwm_channel=3
)

while True:

    print("Engaging braking — actuator should retract...")
    actuator.set_braking(True)
    time.sleep(1.67) # give it time to fully retract

    actuator.keep_pos()
    time.sleep(5)

    print("Disengaging braking — actuator should extend...")
    actuator.set_braking(False)
    time.sleep(5)  # give it time to fully extend

print("Done.")
actuator.close()