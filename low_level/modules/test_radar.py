#!/usr/bin/env python3
"""
HLK-LD2451 radar parser
Parses frames: F4 F3 F2 F1 [lenL] [lenH] [payload...] F8 F7 F6 F5
"""

import struct
from periphery import Serial

# --- Config ---
UART_PORT  = "/dev/ttyAS3"   # adjust to your UART node
BAUD_RATE  = 115200
MAX_PAYLOAD = 128

# Frame delimiters
HDR  = bytes([0xF4, 0xF3, 0xF2, 0xF1])
TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])

# --- Parser state ---
WAIT_HEADER  = 0
READ_LEN     = 1
READ_PAYLOAD = 2
READ_TAIL    = 3


class RadarParser:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state        = WAIT_HEADER
        self.header_index = 0
        self.tail_index   = 0
        self.bytes_read   = 0
        self.frame_len    = 0
        self.len_buf      = bytearray(2)
        self.payload      = bytearray(MAX_PAYLOAD)

    def process_byte(self, b: int):
        if self.state == WAIT_HEADER:
            if b == HDR[self.header_index]:
                self.header_index += 1
                if self.header_index == 4:
                    self.header_index = 0
                    self.state = READ_LEN
            else:
                self.header_index = 0
                if b == HDR[0]:
                    self.header_index = 1  # handle overlapping F4 F4 F3...

        elif self.state == READ_LEN:
            self.len_buf[self.bytes_read] = b
            self.bytes_read += 1

            if self.bytes_read == 2:
                # Little-endian: len = L | (H << 8)
                self.frame_len = struct.unpack_from("<H", self.len_buf)[0]
                self.bytes_read = 0

                if 0 < self.frame_len <= MAX_PAYLOAD:
                    self.state = READ_PAYLOAD
                elif self.frame_len == 0:
                    self.tail_index = 0
                    self.state = READ_TAIL
                else:
                    print("Invalid length, resetting parser")
                    self.reset()

        elif self.state == READ_PAYLOAD:
            self.payload[self.bytes_read] = b
            self.bytes_read += 1

            if self.bytes_read >= self.frame_len:
                self.bytes_read = 0
                self.tail_index = 0
                self.state = READ_TAIL

        elif self.state == READ_TAIL:
            if b == TAIL[self.tail_index]:
                self.tail_index += 1
                if self.tail_index == 4:
                    self.handle_frame()
                    self.reset()
            else:
                self.reset()

    def handle_frame(self):
        print("--------- Frame ---------")
        print(f"Payload length: {self.frame_len}")

        if self.frame_len == 0:
            print("No targets, heartbeat frame.")
            print("-------------------------")
            return

        if self.frame_len < 2:
            print("Invalid payload (too short).")
            print("-------------------------")
            return

        target_count = self.payload[0]
        alarm_info   = self.payload[1]

        print(f"Target count: {target_count}")
        print(f"Alarm info: 0x{alarm_info:02X}")

        has_approach = (alarm_info == 0x01)
        print("Alarm: target approaching" if has_approach else "Alarm: no approaching target")

        # Each target: 5 bytes — angle, distance, direction, speed, SNR
        expected = 2 + target_count * 5
        if self.frame_len < expected:
            print("Warning: payload shorter than expected for target count.")
            print("-------------------------")
            return

        for i in range(target_count):
            base      = 2 + i * 5
            raw_angle = self.payload[base + 0]
            distance  = self.payload[base + 1]
            direction = self.payload[base + 2]  # 0 = away, 1 = approaching
            speed     = self.payload[base + 3]  # km/h
            snr       = self.payload[base + 4]

            # angle = raw_value - 0x80  (signed, per protocol)
            angle_deg = raw_angle - 0x80

            dir_str = {0x00: "away", 0x01: "approaching"}.get(direction, f"unknown (0x{direction:02X})")

            print(f"Target {i + 1}:")
            print(f"  Angle:     {angle_deg} deg")
            print(f"  Distance:  {distance} m")
            print(f"  Direction: {dir_str}")
            print(f"  Speed:     {speed} km/h")
            print(f"  SNR:       {snr}")

        print("-------------------------")


# --- Main ---
def main():
    parser = RadarParser()

    print()
    print("HLK-LD2451 Radar Parser")
    print("Waiting for frames...")

    ser = Serial(UART_PORT, BAUD_RATE)
    try:
        while True:
            data = ser.read(64, timeout=0.1)  # read up to 64 bytes, 1s timeout
            for byte in data:
                parser.process_byte(byte)
    finally:
        ser.close()


if __name__ == "__main__":
    main()