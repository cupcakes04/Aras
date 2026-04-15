import time
from haptic import Haptic

left_haptic  = Haptic('/dev/gpiochip0', 325) #PK5
right_haptic = Haptic('/dev/gpiochip0', 326) #PK6

# When a warning triggers
while True:
    print('fk u')
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