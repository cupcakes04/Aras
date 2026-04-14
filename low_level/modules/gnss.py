"""
gnss.py — NEO-M8N GNSS Library
ARAS (Advanced Rider Assistance System)

Hardware : u-blox NEO-M8N GNSS module
Interface: UART (TTL 3.3 V), default 9600 baud
Platform : Radxa Cubie A7A via python-periphery Serial

Dependencies:
    pip install pynmea2 --break-system-packages

The NEO-M8N outputs standard NMEA 0183 sentences over UART. This library
uses pynmea2 to parse two sentence types:
  $GNGGA / $GPGGA — latitude, longitude, fix quality
  $GNRMC / $GPRMC — latitude, longitude, speed (knots -> km/h)

Update rate is configured to 10 Hz on initialisation via the UBX-CFG-RATE
command, giving a new position fix every 100 ms — suitable for tracking a
motorcycle at highway speeds.
"""

import time
import pynmea2
from periphery import Serial


# ---------------------------------------------------------------------------
# UBX command to set update rate to 10 Hz (measurement period = 100 ms)
# UBX-CFG-RATE: header(2) + class(1) + id(1) + length(2) + payload(6) + checksum(2)
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

# Fix quality values from GGA sentence
FIX_NONE = 0   # no fix
FIX_GPS  = 1   # standard GPS fix
FIX_DGPS = 2   # differential GPS fix


# ---------------------------------------------------------------------------
# GNSS class
# ---------------------------------------------------------------------------
class GNSS:
    """
    Non-blocking driver for the u-blox NEO-M8N GNSS module.

    Maintains an internal line buffer. Each call to read_nmea_data() drains
    available UART bytes, attempts to extract one complete NMEA sentence, and
    returns it immediately. If no complete sentence is ready, None is returned.

    Location and speed are cached internally from the latest valid sentence.
    get_current_location() and get_speed() return the most recently received
    valid values — they do not trigger a new read themselves.

    Call read_nmea_data() on every main loop iteration to keep the cache fresh.

    Parameters
    ----------
    port : str
        UART device path, e.g. '/dev/ttyAS2'
    baudrate : int
        Baud rate (default 9600 matches NEO-M8N factory default).

    Example
    -------
        gnss = GNSS('/dev/ttyAS2')

        while True:
            gnss.read_nmea_data()

            location = gnss.get_current_location()
            speed    = gnss.get_speed()

            if location['fix'] != FIX_NONE:
                print(location['latitude'], location['longitude'])

            if speed is not None:
                print(speed)

        gnss.close()
    """

    def __init__(self, port: str, baudrate: int = 9600):
        self._serial = Serial(port, baudrate)
        self._serial.timeout = 0    # non-blocking reads
        self._buf = ""              # persistent string buffer for incoming bytes

        # Cached values updated by read_nmea_data()
        self._latitude  = None      # decimal degrees, positive = North
        self._longitude = None      # decimal degrees, positive = East
        self._speed_kmh = None      # km/h
        self._fix       = FIX_NONE

        # Send 10 Hz update rate command to the module
        self._set_update_rate_10hz()

    def close(self):
        """Release the UART port."""
        self._serial.close()

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def read_nmea_data(self) -> str:
        """
        Drain available UART bytes and attempt to extract one NMEA sentence.

        Non-blocking: returns immediately whether or not a sentence is ready.
        Automatically updates the internal location and speed cache when a
        valid GGA or RMC sentence is parsed.

        Returns
        -------
        str or None
            The raw NMEA sentence string (e.g. '$GNGGA,123519,...*47') if a
            complete sentence was found. None if no complete sentence is ready.

        Notes
        -----
        The return value is the raw sentence string — useful for logging or
        debugging. For processed values use get_current_location() and
        get_speed() instead.
        """
        try:
            incoming = self._serial.read(256, timeout=0.1)
            if incoming:
                self._buf += incoming.decode("ascii", errors="ignore")
        except Exception:
            pass

        # Drain ALL complete sentences so the buffer never accumulates a backlog.
        # The cache is updated for every sentence; last_sentence holds the most
        # recent one for callers that want the raw string.
        last_sentence = None
        while True:
            newline = self._buf.find("\n")
            if newline == -1:
                break

            sentence = self._buf[:newline].strip()
            self._buf = self._buf[newline + 1:]

            if not sentence.startswith("$"):
                continue

            self._parse_sentence(sentence)
            last_sentence = sentence

        return last_sentence

    def get_current_location(self) -> dict:
        """
        Return the most recently received valid location fix.

        Does not trigger a new UART read — call read_nmea_data() on every
        loop iteration to keep this value current.

        Returns
        -------
        dict
            {
              'fix'       : int,         # FIX_NONE (0), FIX_GPS (1), FIX_DGPS (2)
              'latitude'  : float|None,  # decimal degrees, positive = North
              'longitude' : float|None,  # decimal degrees, positive = East
            }

        Notes
        -----
        latitude and longitude are None if no valid fix has been received yet.
        Always check fix != FIX_NONE before using latitude/longitude values.
        """
        return {
            "fix":       self._fix,
            "latitude":  self._latitude,
            "longitude": self._longitude,
        }

    def get_speed(self) -> float:
        """
        Return the most recently received valid ground speed in km/h.

        Does not trigger a new UART read — call read_nmea_data() on every
        loop iteration to keep this value current.

        Returns
        -------
        float or None
            Speed in km/h. None if no valid RMC sentence has been received yet.

        Notes
        -----
        Speed is derived from the RMC sentence and converted from knots to km/h.
        Always check the return value is not None before using it.
        """
        return self._speed_kmh

    # ------------------------------------------------------------------
    # Private: UBX configuration
    # ------------------------------------------------------------------

    def _set_update_rate_10hz(self) -> None:
        """
        Send the UBX-CFG-RATE command to configure 10 Hz position updates.

        Temporarily switches to blocking mode to ensure the command is sent
        cleanly on startup, then restores non-blocking mode.
        """
        try:
            self._serial.timeout = 0.1
            self._serial.write(_UBX_SET_10HZ)
            time.sleep(0.1)     # allow module time to apply the setting
        except Exception:
            pass
        finally:
            self._serial.timeout = 0

    # ------------------------------------------------------------------
    # Private: NMEA parsing
    # ------------------------------------------------------------------

    def _parse_sentence(self, sentence: str) -> None:
        """
        Parse a raw NMEA sentence using pynmea2 and update the internal cache.

        Handles GGA sentences (fix quality, latitude, longitude) and
        RMC sentences (latitude, longitude, speed).
        """
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
                if msg.status == "A":   # A = active (valid fix)
                    self._latitude  = round(msg.latitude,  6)
                    self._longitude = round(msg.longitude, 6)
                    # pynmea2 spd_over_grnd is in knots, convert to km/h
                    if msg.spd_over_grnd is not None:
                        self._speed_kmh = round(float(msg.spd_over_grnd) * 1.852, 2)

        except pynmea2.ParseError:
            pass
        except Exception:
            pass