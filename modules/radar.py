#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Optional

from periphery import Serial

HDR = bytes([0xF4, 0xF3, 0xF2, 0xF1])
TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])
MAX_PAYLOAD = 128
DEVTTY = "/dev/ttyAS4"

class ParseState:
    WAIT_HEADER = 0
    READ_LEN = 1
    READ_PAYLOAD = 2
    READ_TAIL = 3


@dataclass
class Parser:
    state: int = ParseState.WAIT_HEADER
    header_index: int = 0
    tail_index: int = 0
    frame_len: int = 0
    bytes_read: int = 0

    len_bytes: bytearray = field(default_factory=lambda: bytearray(2))
    payload: bytearray = field(default_factory=lambda: bytearray(MAX_PAYLOAD))
    
    def reset(self) -> None:
        self.state = ParseState.WAIT_HEADER
        self.header_index = 0
        self.tail_index = 0
        self.frame_len = 0
        self.bytes_read = 0

    def process_byte(self, b: int) -> Optional[bytes]:
        if self.state == ParseState.WAIT_HEADER:
            if b == HDR[self.header_index]:
                self.header_index += 1
                if self.header_index == 4:
                    self.header_index = 0
                    self.state = ParseState.READ_LEN
                    self.bytes_read = 0
            else:
                self.header_index = 0
                if b == HDR[0]:
                    self.header_index = 1
            return None

        if self.state == ParseState.READ_LEN:
            self.len_bytes[self.bytes_read] = b
            self.bytes_read += 1
            if self.bytes_read == 2:
                self.frame_len = self.len_bytes[0] | (self.len_bytes[1] << 8)
                self.bytes_read = 0

                if 0 < self.frame_len <= MAX_PAYLOAD:
                    self.state = ParseState.READ_PAYLOAD
                elif self.frame_len == 0:
                    self.state = ParseState.READ_TAIL
                    self.tail_index = 0
                else:
                    print("Invalid length, resetting parser")
                    self.reset()
            return None

        if self.state == ParseState.READ_PAYLOAD:
            if self.bytes_read < MAX_PAYLOAD:
                self.payload[self.bytes_read] = b
            self.bytes_read += 1
            if self.bytes_read >= self.frame_len:
                self.bytes_read = 0
                self.tail_index = 0
                self.state = ParseState.READ_TAIL
            return None

        if self.state == ParseState.READ_TAIL:
            if b == TAIL[self.tail_index]:
                self.tail_index += 1
                if self.tail_index == 4:
                    payload = bytes(self.payload[: self.frame_len])
                    self.reset()
                    return payload
            else:
                self.reset()
            return None

        self.reset()
        return None


def handle_frame(payload: bytes) -> None:
    frame_len = len(payload)
    print("--------- Frame ---------")
    print(f"Payload length: {frame_len}")

    if frame_len == 0:
        print("No targets, heartbeat frame.")
        print("-------------------------")
        return

    if frame_len < 2:
        print("Invalid payload (too short).")
        print("-------------------------")
        return

    target_count = payload[0]
    alarm_info = payload[1]

    print(f"Target count: {target_count}")
    print(f"Alarm info: 0x{alarm_info:02X}")

    if alarm_info == 0x01:
        print("Alarm: target approaching")
    else:
        print("Alarm: no approaching target")

    expected = 2 + target_count * 5
    if frame_len < expected:
        print("Warning: payload shorter than expected for target count.")
        print("-------------------------")
        return

    for i in range(target_count):
        base = 2 + i * 5
        raw_angle = payload[base + 0]
        dist = payload[base + 1]
        direction = payload[base + 2]
        speed = payload[base + 3]
        snr = payload[base + 4]

        angle_deg = int(raw_angle) - 0x80

        print(f"Target {i + 1}:")
        print(f" Angle: {angle_deg} deg")
        print(f" Distance: {dist} m")

        print(" Direction: ", end="")
        if direction == 0x00:
            print("away")
        elif direction == 0x01:
            print("approaching")
        else:
            print(f"unknown (0x{direction:02X})")

        print(f" Speed: {speed} km/h")
        print(f" SNR: {snr}")

    print("-------------------------")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default=DEVTTY)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout-ms", type=int, default=200)
    ap.add_argument("--chunk", type=int, default=256)
    ap.add_argument("--show-heartbeat", action="store_true")
    args = ap.parse_args()

    ser = Serial(
        args.dev,
        baudrate=args.baud,
        databits=8,
        parity="none",
        stopbits=1,
        xonxoff=False,
        rtscts=False,
    )

    parser = Parser()
    print(f"Reading reports from {args.dev} @ {args.baud} ... Ctrl+C to stop")

    try:
        while True:
            data = ser.read(args.chunk, args.timeout_ms)
            if not data:
                continue

            for byte in data:
                payload = parser.process_byte(byte)
                if payload is None:
                    continue

                if len(payload) == 0 and not args.show_heartbeat:
                    continue

                handle_frame(payload)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
