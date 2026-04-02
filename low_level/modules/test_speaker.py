import time
from speaker import Speaker, WARN_FCW, WARN_RCW, WARN_EMERGENCY_BRAKING

speaker = Speaker()

speaker.play(WARN_FCW)
time.sleep(2)
speaker.play(WARN_RCW)
time.sleep(2)
speaker.play(WARN_EMERGENCY_BRAKING)  # interrupts FCW immediately
time.sleep(3)
speaker.stop()
speaker.close()
