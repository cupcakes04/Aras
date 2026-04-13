"""
radar.py — HLK-LD2451 FMCW Radar Library
ARAS (Advanced Rider Assistance System)

Hardware : HLK-LD2451 24 GHz FMCW radar module
Interface: UART (TTL 3.3 V), default 115200 8N1
Platform : Radxa Cubie A7A via python-periphery Serial

Protocol (little-endian, all values hexadecimal):
  Command frame  : FD FC FB FA | len(2B) | cmd_word(2B) + cmd_val(NB) | 04 03 02 01
  ACK frame      : FD FC FB FA | len(2B) | (cmd_word|0x0100)(2B) + ret_val(NB) | 04 03 02 01
  Data frame     : F4 F3 F2 F1 | len(2B) | payload | F8 F7 F6 F5

  Data payload layout:
    Byte 0 : target count (N)
    Byte 1 : alarm info   (0x01 = approaching target present, 0x00 = none)
    Bytes 2+ : N target records, 5 bytes each:
      Byte 0 : angle     — actual degrees = raw - 0x80  (range +-20 degrees)
      Byte 1 : distance  — metres (0-100)
      Byte 2 : direction — 0x00 approaching, 0x01 moving away
      Byte 3 : speed     — km/h (0-120)
      Byte 4 : SNR       — signal-to-noise ratio (0-255)

Important: Any configuration command must be wrapped in enable_config / end_config.
           This is handled internally — callers do not need to manage this.
"""

import struct
import time
from periphery import Serial


# ---------------------------------------------------------------------------
# Frame constants
# ---------------------------------------------------------------------------
CMD_HEADER  = bytes([0xFD, 0xFC, 0xFB, 0xFA])
CMD_FOOTER  = bytes([0x04, 0x03, 0x02, 0x01])
DATA_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
DATA_FOOTER = bytes([0xF8, 0xF7, 0xF6, 0xF5])

# Command words
CMD_ENABLE_CONFIG   = 0x00FF
CMD_END_CONFIG      = 0x00FE
CMD_SET_DETECTION   = 0x0002
CMD_GET_DETECTION   = 0x0012
CMD_SET_SENSITIVITY = 0x0003
CMD_GET_SENSITIVITY = 0x0013
CMD_SET_BAUD        = 0x00A1
CMD_FACTORY_RESET   = 0x00A2
CMD_RESTART         = 0x00A3

# Baud rate index map (Table 6 in protocol doc)
BAUD_INDEX = {
    9600:   0x0001,
    19200:  0x0002,
    38400:  0x0003,
    57600:  0x0004,
    115200: 0x0005,
    230400: 0x0006,
    256000: 0x0007,
    460800: 0x0008,
}

# Direction constants for set_detection_params()
DIR_DETECT_AWAY     = 0x00   # only detect vehicles moving away
DIR_DETECT_APPROACH = 0x01   # only detect approaching vehicles
DIR_DETECT_ALL      = 0x02   # detect both directions

# Human-readable direction strings returned in target dicts
DIR_APPROACHING = "approaching"
DIR_MOVING_AWAY = "moving_away"

# Default timeout (seconds) used for blocking config ACK reads
_CONFIG_TIMEOUT = 2.0


# ---------------------------------------------------------------------------
# Helper: build a command frame
# ---------------------------------------------------------------------------
def _build_cmd(cmd_word: int, cmd_value: bytes = b"") -> bytes:
    """Pack a command word and optional value into the LD2451 command frame format."""
    intra  = struct.pack("<H", cmd_word) + cmd_value
    length = struct.pack("<H", len(intra))
    return CMD_HEADER + length + intra + CMD_FOOTER


# ---------------------------------------------------------------------------
# Radar class
# ---------------------------------------------------------------------------
class Radar:
    """
    Driver for the HLK-LD2451 24 GHz FMCW radar module.

    Reading methods (read_data, get_targets) are non-blocking and designed
    to be called on every main loop iteration.

    Configuration methods (set_detection_params, set_sensitivity, etc.) are
    blocking — they send a command and wait for the radar's ACK. They should
    be called once during initialisation, not inside the main loop.

    Parameters
    ----------
    port : str
        UART device path, e.g. '/dev/ttyAS1'
    baudrate : int
        Must match the module's current baud rate setting (factory default 115200).

    Example
    -------
        radar = Radar('/dev/ttyAS1')

        # Configure once at startup
        radar.set_detection_params(max_distance=80, direction=DIR_DETECT_APPROACH)
        radar.set_sensitivity(trigger_count=2, snr_threshold=4)

        # Poll on every loop iteration
        while True:
            result = radar.get_targets()
            if result['alarm']:
                for t in result['targets']:
                    print(t['distance'], t['speed'])

        radar.close()
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self._serial = Serial(port, baudrate)
        self._serial.timeout = 0    # non-blocking for data reads
        self._buf = bytearray()     # persistent receive buffer for data frames

    def close(self):
        """Release the UART port."""
        self._serial.close()

    # ------------------------------------------------------------------
    # Reading methods (non-blocking)
    # ------------------------------------------------------------------

    def read_data(self) -> dict:
        """
        Drain available UART bytes and attempt to parse one radar data frame.

        Non-blocking: returns immediately whether or not a full frame is ready.

        Returns
        -------
        dict or None
            Returns a parsed frame dict if a complete, valid frame was found
            in the buffer. Returns None if no complete frame is available yet
            (call again on the next loop iteration).

        Return dict structure
        ---------------------
        {
          'alarm'   : bool,        # True if at least one approaching target is present
          'count'   : int,         # total number of detected targets
          'targets' : list[dict]   # one dict per target (empty list if count=0)
        }

        Target dict fields
        ------------------
        'angle'     : int  — signed degrees; positive = right of boresight,
                             negative = left. Range +-20 degrees.
        'distance'  : int  — metres (0-100)
        'direction' : str  — 'approaching' or 'moving_away'
        'speed'     : int  — km/h (0-120)
        'snr'       : int  — signal-to-noise ratio (0-255)
        """
        try:
            incoming = self._serial.read(256, timeout = 0.1)
            if incoming:
                self._buf += incoming
        except Exception:
            pass

        return self._extract_frame()

    def get_targets(self) -> dict:
        """
        Non-blocking read that always returns a well-formed dict.

        Wraps read_data() and guarantees a safe return structure even when
        no frame is available, so main.py can always access result['targets']
        without checking for None.

        Returns
        -------
        dict
            {
              'alarm'   : bool,       # True if at least one approaching target detected
              'count'   : int,        # 0 if no frame available or no targets
              'targets' : list[dict]  # empty list if no frame or no targets
            }

        Notes
        -----
        A return of alarm=False / count=0 / targets=[] means either:
          (a) the radar genuinely sees no targets this cycle, or
          (b) a complete frame has not arrived yet.
        Call on every main loop iteration for continuous updates.
        """
        result = self.read_data()
        if result is None:
            return {"alarm": False, "count": 0, "targets": []}
        return result

    # ------------------------------------------------------------------
    # Configuration methods (blocking, call during initialisation only)
    # ------------------------------------------------------------------

    def set_detection_params(
        self,
        max_distance: int = 100,
        direction: int = DIR_DETECT_ALL,
        min_speed: int = 5,
        no_target_delay: int = 2,
    ) -> bool:
        """
        Configure target detection parameters (cmd 0x0002).

        Parameters
        ----------
        max_distance : int
            Maximum detection range in metres (10-255).
        direction : int
            DIR_DETECT_AWAY (0x00)  — only targets moving away from the radar.
            DIR_DETECT_APPROACH (0x01) — only approaching targets.
            DIR_DETECT_ALL (0x02)   — both directions (default).
        min_speed : int
            Minimum target speed in km/h to trigger a detection (0-120).
            Targets moving slower than this are ignored.
        no_target_delay : int
            Seconds the radar continues reporting after a target disappears (0-255).

        Returns
        -------
        bool — True if radar acknowledged success, False otherwise.
        """
        cmd_value = bytes([max_distance, direction, min_speed, no_target_delay])
        return self._run_config_command(CMD_SET_DETECTION, cmd_value)

    def get_detection_params(self) -> dict:
        """
        Read the radar's current detection parameters (cmd 0x0012).

        Returns
        -------
        dict or None
            {
              'max_distance'    : int,  # metres
              'direction'       : int,  # 0=away only, 1=approach only, 2=all
              'min_speed'       : int,  # km/h
              'no_target_delay' : int   # seconds
            }
        Returns None if the radar does not respond or returns a failure ACK.
        """
        data = self._run_config_command_with_data(CMD_GET_DETECTION, b"")
        if data is None or len(data) < 4:
            return None
        return {
            "max_distance":    data[0],
            "direction":       data[1],
            "min_speed":       data[2],
            "no_target_delay": data[3],
        }

    def set_sensitivity(
        self,
        trigger_count: int = 1,
        snr_threshold: int = 4,
    ) -> bool:
        """
        Configure detection sensitivity (cmd 0x0003).

        Parameters
        ----------
        trigger_count : int
            Number of consecutive detections required before a target is reported
            (1-10). Higher = fewer false positives but slightly slower response.
            Default is 1.
        snr_threshold : int
            Signal-to-noise ratio threshold (3-8 recommended; default 4).
            Lower = more sensitive (easier to trigger).
            Higher = less sensitive (harder to trigger).

        Returns
        -------
        bool — True on success.
        """
        cmd_value = bytes([trigger_count, snr_threshold, 0x00, 0x00])
        return self._run_config_command(CMD_SET_SENSITIVITY, cmd_value)

    def get_sensitivity(self) -> dict:
        """
        Read the radar's current sensitivity settings (cmd 0x0013).

        Returns
        -------
        dict or None
            {
              'trigger_count' : int,
              'snr_threshold' : int
            }
        Returns None on failure.
        """
        data = self._run_config_command_with_data(CMD_GET_SENSITIVITY, b"")
        if data is None or len(data) < 2:
            return None
        return {
            "trigger_count": data[0],
            "snr_threshold": data[1],
        }

    def set_baud_rate(self, baudrate: int) -> bool:
        """
        Change the radar module's serial baud rate (cmd 0x00A1).

        The new rate is saved to flash and takes effect after a restart.
        Remember to update the baudrate argument passed to LD2451Radar()
        after restarting.

        Supported values: 9600, 19200, 38400, 57600, 115200,
                          230400, 256000, 460800.

        Parameters
        ----------
        baudrate : int
            Desired baud rate. Must be one of the supported values above.

        Returns
        -------
        bool — True on success, False if baudrate is unsupported or radar rejected.
        """
        idx = BAUD_INDEX.get(baudrate)
        if idx is None:
            return False
        cmd_value = struct.pack("<H", idx)
        return self._run_config_command(CMD_SET_BAUD, cmd_value)

    def factory_reset(self) -> bool:
        """
        Restore all radar settings to factory defaults (cmd 0x00A2).

        Settings take effect after a restart. Equivalent to calling
        factory_reset() followed by restart().

        Returns
        -------
        bool — True on success.
        """
        return self._run_config_command(CMD_FACTORY_RESET, b"")

    def restart(self) -> bool:
        """
        Restart the radar module (cmd 0x00A3).

        The radar sends an ACK then reboots. Allow ~1 second before
        sending further commands or expecting data output.

        Returns
        -------
        bool — True if ACK was received before the reboot.
        """
        return self._run_config_command(CMD_RESTART, b"")

    # ------------------------------------------------------------------
    # Private: data frame parsing
    # ------------------------------------------------------------------

    def _extract_frame(self) -> dict:
        """
        Scan the internal buffer for a complete, valid data output frame.

        Discards leading garbage bytes before DATA_HEADER.
        Returns a parsed dict and removes the frame from the buffer on success.
        Returns None without modifying the buffer if the frame is incomplete.
        Drops the header byte and retries if the footer doesn't match (corrupt).
        """
        while True:
            idx = self._buf.find(DATA_HEADER)
            if idx == -1:
                self._buf = self._buf[-3:]
                return None

            if idx > 0:
                self._buf = self._buf[idx:]

            if len(self._buf) < 6:
                return None

            payload_len = struct.unpack_from("<H", self._buf, 4)[0]
            total_len = 4 + 2 + payload_len + 4

            if len(self._buf) < total_len:
                return None

            frame = bytes(self._buf[:total_len])
            if frame[-4:] != DATA_FOOTER:
                self._buf = self._buf[1:]
                continue

            self._buf = self._buf[total_len:]
            payload = frame[6: 6 + payload_len]
            return self._parse_payload(payload)

    def _parse_payload(self, payload: bytes) -> dict:
        """Decode payload bytes from a validated data frame into a target dict."""
        if len(payload) < 2:
            return {"alarm": False, "count": 0, "targets": []}

        count = payload[0]
        alarm = (payload[1] == 0x01)
        targets = []

        for i in range(count):
            offset = 2 + i * 5
            if offset + 5 > len(payload):
                break

            targets.append({
                "angle":     payload[offset]     - 0x80,
                "distance":  payload[offset + 1],
                "direction": DIR_APPROACHING if payload[offset + 2] == 0x00 else DIR_MOVING_AWAY,
                "speed":     payload[offset + 3],
                "snr":       payload[offset + 4],
            })

        return {"alarm": alarm, "count": count, "targets": targets}

    # ------------------------------------------------------------------
    # Private: configuration command infrastructure
    # ------------------------------------------------------------------

    def _send_cmd(self, cmd_word: int, cmd_value: bytes = b"") -> None:
        """Write a command frame to the UART."""
        self._serial.write(_build_cmd(cmd_word, cmd_value))

    def _read_ack(self, expected_cmd_word: int) -> bytes:
        """
        Blocking read for a command ACK frame.

        Switches the serial port to blocking mode temporarily, reads bytes
        until a matching ACK frame is found or the timeout expires, then
        restores non-blocking mode.

        Returns the return-value bytes after the 2-byte ACK status on success,
        or None on timeout or failure status.
        """
        expected_ack_word = expected_cmd_word | 0x0100
        deadline = time.monotonic() + _CONFIG_TIMEOUT
        ack_buf = bytearray()

        # Temporarily switch to blocking reads for the ACK
        self._serial.timeout = 0.05

        try:
            while time.monotonic() < deadline:
                try:
                    chunk = self._serial.read(64, timeout = 0.05)
                except Exception:
                    chunk = b""
                if chunk:
                    ack_buf += chunk

                if len(ack_buf) < 4:
                    continue

                idx = ack_buf.find(CMD_HEADER)
                if idx == -1:
                    ack_buf = ack_buf[-3:]
                    continue
                if idx > 0:
                    ack_buf = ack_buf[idx:]

                if len(ack_buf) < 6:
                    continue

                payload_len = struct.unpack_from("<H", ack_buf, 4)[0]
                total_len = 4 + 2 + payload_len + 4

                if len(ack_buf) < total_len:
                    continue

                frame = bytes(ack_buf[:total_len])
                if frame[-4:] != CMD_FOOTER:
                    ack_buf = ack_buf[1:]
                    continue

                payload = frame[6: 6 + payload_len]

                if len(payload) < 4:
                    ack_buf = ack_buf[1:]
                    continue

                ack_word   = struct.unpack_from("<H", payload, 0)[0]
                ack_status = struct.unpack_from("<H", payload, 2)[0]

                if ack_word != expected_ack_word:
                    # Not our ACK (could be a stray data frame) — skip past it
                    ack_buf = ack_buf[total_len:]
                    continue

                if ack_status != 0x0000:
                    return None  # Radar reported failure

                return payload[4:]  # Return any extra data bytes

        finally:
            self._serial.timeout = 0  # Restore non-blocking mode

        return None

    def _enable_config(self) -> bool:
        """Send the enable-configuration command and await ACK."""
        self._send_cmd(CMD_ENABLE_CONFIG, struct.pack("<H", 0x0001))
        return self._read_ack(CMD_ENABLE_CONFIG) is not None

    def _end_config(self) -> bool:
        """Send the end-configuration command and await ACK."""
        self._send_cmd(CMD_END_CONFIG)
        return self._read_ack(CMD_END_CONFIG) is not None

    def _run_config_command(self, cmd_word: int, cmd_value: bytes) -> bool:
        """
        Full config sequence: enable → send command → end.
        Returns True only if all three ACKs succeed.
        """
        if not self._enable_config():
            return False
        self._send_cmd(cmd_word, cmd_value)
        if self._read_ack(cmd_word) is None:
            self._end_config()
            return False
        return self._end_config()

    def _run_config_command_with_data(
        self, cmd_word: int, cmd_value: bytes
    ) -> bytes:
        """
        Full config sequence for commands that return data in the ACK payload.
        Returns the extra ACK data bytes, or None on failure.
        """
        if not self._enable_config():
            return None
        self._send_cmd(cmd_word, cmd_value)
        data = self._read_ack(cmd_word)
        self._end_config()
        return data