import asyncio
import random
import time
from collections import deque
from .history import History

class RadarOld(History):
    """
    Radar
    - functions in reading every detected objects 
    - data of 
        - angle  : degrees, -90 (left) to +90 (right)
        - distance: metres
        - direction: True = approaching, False = receding
        - speed  : km/h (relative to radar/bike)
        - snr    : 0.0-1.0 normalised signal-to-noise ratio (confidence)
    
    Example output:
    ```python
        [
            {'angle': 30,  'distance': 10, 'direction': True,  'speed': 50, 'snr': 0.6},
            {'angle': -15, 'distance': 5,  'direction': False, 'speed': 10, 'snr': 0.9},
        ]
    ```
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def read(self):
        # 1. Read the sensor values (placeholder)
        value = [
            {'angle': 30,  'distance': 10, 'direction': True,  'speed': 2, 'snr': 0.6},
            {'angle': -15, 'distance': 5,  'direction': False, 'speed': 1, 'snr': 0.9},
        ]

        # 2. Record the action in history
        self.save_history(value)


# ---------------------------------------------------------------------------
# Frame constants
# ---------------------------------------------------------------------------
CMD_HEADER  = bytes([0xFD, 0xFC, 0xFB, 0xFA])
CMD_FOOTER  = bytes([0x04, 0x03, 0x02, 0x01])
DATA_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
DATA_FOOTER = bytes([0xF8, 0xF7, 0xF6, 0xF5])

CMD_ENABLE_CONFIG   = 0x00FF
CMD_END_CONFIG      = 0x00FE
CMD_SET_DETECTION   = 0x0002
CMD_GET_DETECTION   = 0x0012
CMD_SET_SENSITIVITY = 0x0003
CMD_GET_SENSITIVITY = 0x0013
CMD_SET_BAUD        = 0x00A1
CMD_FACTORY_RESET   = 0x00A2
CMD_RESTART         = 0x00A3

DIR_DETECT_AWAY     = 0x00
DIR_DETECT_APPROACH = 0x01
DIR_DETECT_ALL      = 0x02

def _build_cmd(cmd_word: int, cmd_value: bytes = b"") -> bytes:
    intra  = struct.pack("<H", cmd_word) + cmd_value
    length = struct.pack("<H", len(intra))
    return CMD_HEADER + length + intra + CMD_FOOTER

class Radar(History):
    """
    Driver for the HLK-LD2451 24 GHz FMCW radar module via UART.
    Integrated into system.py History structure.
    """

    def __init__(self, port: str = "/dev/ttyS3", baudrate: int = 115200, **kwargs):
        super().__init__(**kwargs)
        self.setup(port, baudrate)

    def setup(self, port: str, baudrate: int):
        self._hardware_available = True
        try:
            self._serial = Serial(port, baudrate)
        except Exception as e:
            print(f"[Radar {port}] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False
            
        self._buffer = bytearray()
        
    def close(self):
        if self._hardware_available:
            self._serial.close()

    def _read_data_frame(self) -> dict:
        """Non-blocking read of one data frame from the UART buffer."""
        if not self._hardware_available:
            return None
            
        while True:
            # Read whatever is available
            chunk = self._serial.read(256, timeout=0.0)
            if chunk:
                self._buffer.extend(chunk)
            else:
                break

        # Need at least 10 bytes for header, length, footer
        while len(self._buffer) >= 10:
            header_idx = self._buffer.find(DATA_HEADER)
            if header_idx == -1:
                # No header found, clear buffer
                self._buffer.clear()
                return None
                
            if header_idx > 0:
                # Discard garbage before header
                self._buffer = self._buffer[header_idx:]

            if len(self._buffer) < 6:
                return None # Wait for more data

            # Extract length (2 bytes, little-endian)
            data_len = struct.unpack("<H", self._buffer[4:6])[0]
            frame_size = 6 + data_len + 4

            if len(self._buffer) < frame_size:
                return None # Wait for full frame

            # Extract frame
            frame = self._buffer[:frame_size]
            self._buffer = self._buffer[frame_size:]

            # Verify footer
            if frame[-4:] != DATA_FOOTER:
                continue # Bad frame, find next header
                
            payload = frame[6:-4]
            return self._parse_payload(payload)
            
        return None

    def _parse_payload(self, payload: bytes) -> dict:
        if len(payload) < 2:
            return {'target_count': 0, 'alarm': False, 'targets': []}

        target_count = payload[0]
        alarm = payload[1] == 0x01
        
        targets = []
        offset = 2
        
        for _ in range(target_count):
            if offset + 5 > len(payload):
                break # Malformed
                
            record = payload[offset:offset+5]
            offset += 5
            
            angle     = record[0] - 0x80
            distance  = record[1]
            direction = record[2] == 0x00 # 0x00 is approaching (True in our format)
            speed     = record[3]
            snr       = record[4] / 255.0 # Normalise 0-255 to 0.0-1.0
            
            targets.append({
                'angle': angle,
                'distance': distance,
                'direction': direction,
                'speed': speed,
                'snr': snr,
            })
            
        return {'target_count': len(targets), 'alarm': alarm, 'targets': targets}

    async def read(self):
        """
        Polls the serial buffer, extracts all complete frames, 
        and saves the targets from the *latest* valid frame to history.
        """
        if not self._hardware_available:
            # Provide dummy target if hardware missing
            value = [
                {'angle': 0, 'distance': 10, 'direction': True, 'speed': 5, 'snr': 0.5}
            ]
            self.save_history(value)
            return

        latest_targets = None
        
        while True:
            data = self._read_data_frame()
            if data is None:
                break
            latest_targets = data['targets']
            
        if latest_targets is not None:
            # Our system expects a list of targets (dict format)
            self.save_history(latest_targets)