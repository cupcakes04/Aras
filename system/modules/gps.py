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
            "speed_kmh": 44.6 + random.uniform(-1.0, 1.0),
            "speed_limit": 60
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

    Maintains an internal line buffer. Each call to _read_nmea_data() reads
    up to 256 bytes, drains all complete NMEA sentences from the buffer, and
    updates the internal cache. Falls back to dummy/last-known values if
    hardware is unavailable.
    """

    def __init__(self, port: str = '/dev/ttyAS4', baudrate: int = 9600, **kwargs):
        super().__init__(**kwargs)
        self.setup(port, baudrate)

    def setup(self, port: str, baudrate: int):
        self._hardware_available = True
        try:
            self._serial = Serial(port, baudrate)
            self._serial.timeout = 0    # non-blocking reads by default
            self._set_update_rate_10hz()
        except Exception as e:
            print(f"[GPS {port}] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False

        self._buf = ""
        self._latitude  = 2.946493
        self._longitude = 101.875237
        self._speed_kmh = 0.0
        self._fix       = FIX_NONE

    def close(self):
        if self._hardware_available:
            self._serial.close()

    # ------------------------------------------------------------------
    # Private: UBX configuration
    # ------------------------------------------------------------------

    def _set_update_rate_10hz(self) -> None:
        """Send UBX-CFG-RATE to configure 10 Hz position updates."""
        try:
            self._serial.timeout = 0.1
            self._serial.write(_UBX_SET_10HZ)
            time.sleep(0.1)     # allow module time to apply the setting
        except Exception:
            pass
        finally:
            self._serial.timeout = 0

    # ------------------------------------------------------------------
    # Private: NMEA reading and parsing
    # ------------------------------------------------------------------

    def _read_nmea_data(self):
        if not self._hardware_available:
            return None

        # Read one chunk then drain — avoids blocking indefinitely on a live stream
        try:
            incoming = self._serial.read(256, timeout=0.1)
            if incoming:
                self._buf += incoming.decode('ascii', errors='ignore')
        except Exception:
            pass

        # Drain ALL complete sentences so the buffer never accumulates a backlog
        while '\n' in self._buf:
            line, self._buf = self._buf.split('\n', 1)
            line = line.strip()
            if not line.startswith('$'):
                continue
            self._parse_sentence(line)

    def _parse_sentence(self, sentence: str) -> None:
        """Parse one NMEA sentence and update the internal cache."""
        try:
            msg = pynmea2.parse(sentence)

            # GGA — fix quality, latitude, longitude
            if isinstance(msg, pynmea2.GGA):
                self._fix = int(msg.gps_qual) if msg.gps_qual else FIX_NONE
                if self._fix != FIX_NONE:
                    self._latitude  = round(msg.latitude,  6)
                    self._longitude = round(msg.longitude, 6)

            # RMC — latitude, longitude, speed
            elif isinstance(msg, pynmea2.RMC):
                if msg.status == 'A':   # A = active (valid fix)
                    self._latitude  = round(msg.latitude,  6)
                    self._longitude = round(msg.longitude, 6)
                    if msg.spd_over_grnd is not None:
                        self._speed_kmh = round(float(msg.spd_over_grnd) * 1.852, 2)

        except pynmea2.ParseError:
            print('parse ewrrro')
            pass
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Async interface (called by system loop)
    # ------------------------------------------------------------------

    async def read(self):
        self._read_nmea_data()

        # Returns last-known values if hardware unavailable or no fix yet
        value = {
            "latitude":  self._latitude,
            "longitude": self._longitude,
            "speed_kmh": self._speed_kmh,
            "fix":       self._fix,
            "speed_limit": 60  # Default or placeholder speed limit
        }

        self.save_history(value)