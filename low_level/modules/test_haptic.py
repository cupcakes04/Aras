import time
from haptic import Haptic

left_haptic  = Haptic('/dev/gpiochip0', 323) #PK3
right_haptic = Haptic('/dev/gpiochip0', 324) #PK4

# When a warning triggers
while True:
    left_haptic.on()
    time.sleep(3)
    right_haptic.on()
    time.sleep(3) 
    left_haptic.on()
    right_haptic.on()
    time.sleep(1)
    left_haptic.off()
    right_haptic.off()
    time.sleep(1)
    left_haptic.on()
    right_haptic.on()
    time.sleep(1)

# At the end of the program
left_haptic.close()
right_haptic.close()