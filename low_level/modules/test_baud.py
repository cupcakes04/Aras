# test_baud.py
from periphery import Serial
import time

PORT = '/dev/ttyAS3'
BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]

for baud in BAUDS:
    try:
        uart = Serial(PORT, baud)
        uart.write(b'LOOPBACK_TEST')
        time.sleep(0.2)
        received = uart.read(64, timeout=1)
        uart.close()
        print(f"{baud:>7} baud — {received}")
    except Exception as e:
        print(f"{baud:>7} baud — ERROR: {e}")
    time.sleep(0.3)