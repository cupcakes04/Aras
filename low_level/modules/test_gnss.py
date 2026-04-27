# main.py
import time
from gnss import GNSS, FIX_NONE

# --- Initialize ---
gnss = GNSS('/dev/ttyAS4', baudrate=9600)

print("=============================")
print("  ARAS GNSS Module Ready     ")
print("=============================")
print("Waiting for GPS fix...\n")

try:
    last_print = 0

    while True:
        # Call every loop iteration to keep cache fresh (non-blocking)
        gnss.read_nmea_data()

        # Print every 1 second (module updates at 10 Hz internally)
        if time.monotonic() - last_print >= 1.0:
            last_print = time.monotonic()

            location = gnss.get_current_location()
            speed    = gnss.get_speed()

            print("-----------------------------")

            if location['fix'] != FIX_NONE:
                print(f"Latitude  : {location['latitude']}") # float
                print(f"Longitude : {location['longitude']}") # float
                print(f"Fix Type  : {'DGPS' if location['fix'] == 2 else 'GPS'}")
            else:
                print("Location  : No Fix")

            if speed is not None:
                print(f"Speed     : {speed} km/h") # float
            else:
                print("Speed     : No Fix")

            print()

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    gnss.close()