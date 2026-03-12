from periphery import Serial
import time
import os

# ---- EDIT THESE ----
GPS_DEV = "/dev/ttyAS4"
GPS_BAUD = 9600
# --------------------

KNOTS_TO_KMH = 1.852

# Tuning knobs
MIN_SATS = 6
MAX_HDOP = 3.0
STOP_THRESHOLD_KMH = 2.0
EMA_ALPHA = 0.2


def read_cpu_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            raw = f.read().strip()
        return int(raw) / 1000.0
    except Exception:
        return None


def parse_gga(line: str):
    parts = line.split(",")
    if len(parts) < 10:
        return None

    try:
        fix_q = int(parts[6]) if parts[6] else 0
    except ValueError:
        fix_q = 0

    try:
        sats = int(parts[7]) if parts[7] else 0
    except ValueError:
        sats = 0

    try:
        hdop = float(parts[8]) if parts[8] else 99.99
    except ValueError:
        hdop = 99.99

    return fix_q, sats, hdop


def parse_speed_kmh(line: str):
    if line.startswith(("$GNVTG", "$GPVTG")):
        parts = line.split(",")
        if len(parts) > 9 and parts[7]:
            try:
                return float(parts[7])
            except ValueError:
                return None

    if line.startswith(("$GNRMC", "$GPRMC")):
        parts = line.split(",")
        if len(parts) > 7:
            status = parts[2].strip()
            if status == "A" and parts[7]:
                try:
                    return float(parts[7]) * KNOTS_TO_KMH
                except ValueError:
                    return None

    return None


def draw_screen(fix_q, sats, hdop, speed_kmh_display, good_fix, cpu_temp_c):
    fix_str = "NO FIX" if fix_q == 0 else f"FIX({fix_q})"
    temp_str = "--.-C" if cpu_temp_c is None else f"{cpu_temp_c:4.1f}C"
    speed_str = "--" if not good_fix else f"{speed_kmh_display:5.2f}"

    # Move cursor to top-left and overwrite lines in place
    print("\033[H", end="")
    print(f"GPS: {fix_str:<12} CPU: {temp_str}")
    print(f"Sats:{sats:2d}  HDOP:{hdop:4.2f}          ")
    print(f"Speed: {speed_str} km/h          ")
    print("(Ctrl+C to exit)    ", end="", flush=True)


def main():
    ser = Serial(GPS_DEV, GPS_BAUD)
    buf = bytearray()

    fix_q, sats, hdop = 0, 0, 99.99
    speed_ema = None
    last_draw = 0.0

    # Clear screen once at startup
    print("\033[2J\033[H", end="")
    print(f"GPS:  {GPS_DEV} @ {GPS_BAUD}")
    print("Waiting for data...\n")

    try:
        while True:
            data = ser.read(256, timeout=1.0)
            if data:
                buf.extend(data)

                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)

                    s = line.decode("ascii", errors="ignore").strip()
                    if not s.startswith("$"):
                        continue
                    if s.startswith(("$GNTXT", "$GPTXT")):
                        continue

                    if s.startswith(("$GNGGA", "$GPGGA")):
                        g = parse_gga(s)
                        if g:
                            fix_q, sats, hdop = g

                    sp = parse_speed_kmh(s)
                    if sp is not None:
                        speed_ema = sp if speed_ema is None else (EMA_ALPHA * sp + (1 - EMA_ALPHA) * speed_ema)

            now = time.time()
            if now - last_draw >= 0.25:
                last_draw = now

                cpu_temp_c = read_cpu_temp_c()
                good_fix = (fix_q >= 1) and (sats >= MIN_SATS) and (hdop <= MAX_HDOP)

                if good_fix and speed_ema is not None:
                    speed_disp = 0.0 if speed_ema < STOP_THRESHOLD_KMH else speed_ema
                else:
                    speed_disp = 0.0

                draw_screen(fix_q, sats, hdop, speed_disp, good_fix, cpu_temp_c)

    except KeyboardInterrupt:
        print("\n\nExited.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()