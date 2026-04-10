import asyncio
import random
import time
import pynmea2
from collections import deque
from periphery import Serial
from .history import History

class GPSOld(History):
    """
    GPS
    - functions in reafing satelite data
    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        {
            "latitude": 3.1390, [-90,90]
            "longitude": 101.6869, [-180,180]
            "speed_kmh": 44.6 kmh
        }

    ```

    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        pass
        
    async def read(self):
        value = {
            "latitude": 3.1390 + random.uniform(-0.0001, 0.0001),
            "longitude": 101.6869 + random.uniform(-0.0001, 0.0001),
            "speed_kmh": 44.6 + random.uniform(-1.0, 1.0)
        }
        self.save_history(value)


# ---------------------------------------------------------------------------
# UBX command to set update rate to 10 Hz (measurement period = 100 ms)
# ---------------------------------------------------------------------------
_UBX_SET_10HZ = bytes([
    0xB5, 0x62,             # UBX header
    0x06, 0x08,             # class CFG, id RATE
    0x06, 0x00,             # payload length = 6 bytes
    0x64, 0x00,             # measRate = 100 ms (little-endian) -> 10 Hz
    0x01, 0x00,             # navRate = 1 (navigate every measurement)
    0x01, 0x00,             # timeRef = 1 (GPS time)
    0x7A, 0x12,             # checksum (CK_A, CK_B)
])

FIX_NONE = 0
FIX_GPS  = 1
FIX_DGPS = 2

class GPS(History):
    """
    Non-blocking driver for the u-blox NEO-M8N GNSS module via UART.
    Integrated into system.py History structure.
    """

    def __init__(self, port: str = '/dev/ttyS4', baudrate: int = 9600, **kwargs):
        super().__init__(**kwargs)
        self.setup(port, baudrate)

    def setup(self, port: str, baudrate: int):
        self._hardware_available = True
        try:
            self._serial = Serial(port, baudrate)
            self._serial.timeout = 0
            # Set to 10Hz
            self._serial.write(_UBX_SET_10HZ)
        except Exception as e:
            print(f"[GPS {port}] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False
            
        self._buf = ""
        self._latitude  = 3.1390
        self._longitude = 101.6869
        self._speed_kmh = 0.0
        self._fix       = FIX_NONE

    def close(self):
        if self._hardware_available:
            self._serial.close()

    def _read_nmea_data(self):
        if not self._hardware_available:
            return None

        # Drain all available bytes from UART
        while True:
            chunk = self._serial.read(256)
            if not chunk:
                break
            try:
                self._buf += chunk.decode('ascii')
            except UnicodeDecodeError:
                # Discard bad bytes
                pass

        # Parse sentences from buffer
        while '\n' in self._buf:
            line, self._buf = self._buf.split('\n', 1)
            line = line.strip()
            
            if not line.startswith('$'):
                continue
                
            try:
                msg = pynmea2.parse(line)
                
                # GGA: Fix data (lat, lon, fix quality)
                if msg.sentence_type == 'GGA':
                    self._fix = msg.gps_qual
                    if self._fix != FIX_NONE and msg.latitude != 0.0:
                        self._latitude  = msg.latitude
                        self._longitude = msg.longitude

                # RMC: Recommended minimum (lat, lon, speed)
                elif msg.sentence_type == 'RMC':
                    if msg.status == 'A' and msg.latitude != 0.0: # A = active/valid
                        self._latitude  = msg.latitude
                        self._longitude = msg.longitude
                        if msg.spd_over_grnd is not None:
                            # Knots to km/h
                            self._speed_kmh = float(msg.spd_over_grnd) * 1.852

            except pynmea2.ParseError:
                pass

    async def read(self):
        self._read_nmea_data()
        
        # If hardware fails or no fix, we return the last known (or default) values
        value = {
            "latitude": self._latitude,
            "longitude": self._longitude,
            "speed_kmh": self._speed_kmh,
            "fix": self._fix
        }
        
        self.save_history(value)