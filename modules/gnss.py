from periphery import Serial
import time

DEV = "/dev/ttyAS4"
BAUD = 9600

KNOTS_TO_KMH = 1.852

# Tuning knobs
MIN_SATS = 6
MAX_HDOP = 2.5
STOP_THRESHOLD_KMH = 2.0   # raise if you're indoors
EMA_ALPHA = 0.2            # lower = smoother, slower response


def parse_gga(line: str):
    # $GNGGA,time,lat,N,lon,E,fix,numsats,hdop,alt,M,...
    parts = line.split(",")
    if len(parts) < 10:
        return None
    fix_q = parts[6].strip()
    sats = parts[7].strip()
    hdop = parts[8].strip()

    try:
        fix_q_i = int(fix_q) if fix_q else 0
    except ValueError:
        fix_q_i = 0

    try:
        sats_i = int(sats) if sats else 0
    except ValueError:
        sats_i = 0

    try:
        hdop_f = float(hdop) if hdop else 99.99
    except ValueError:
        hdop_f = 99.99

    return fix_q_i, sats_i, hdop_f


def parse_speed_kmh(line: str):
    # VTG preferred (km/h)
    if line.startswith(("$GNVTG", "$GPVTG")):
        parts = line.split(",")
        if len(parts) > 9:
            sp_kmh = parts[7].strip()
            if sp_kmh:
                try:
                    return float(sp_kmh)
                except ValueError:
                    return None

    # RMC fallback (knots)
    if line.startswith(("$GNRMC", "$GPRMC")):
        parts = line.split(",")
        if len(parts) > 7:
            status = parts[2].strip()  # A=valid, V=void
            sog_knots = parts[7].strip()
            if status == "A" and sog_knots:
                try:
                    return float(sog_knots) * KNOTS_TO_KMH
                except ValueError:
                    return None
    return None


def main():
    ser = Serial(DEV, BAUD)
    buf = bytearray()

    fix_q, sats, hdop = 0, 0, 99.99
    speed_raw = None
    speed_ema = None
    last_print = 0.0

    print(f"Reading GPS on {DEV} @ {BAUD}")
    print("Ctrl+C to exit")

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
                        speed_raw = sp

                        # Smooth
                        if speed_ema is None:
                            speed_ema = speed_raw
                        else:
                            speed_ema = (EMA_ALPHA * speed_raw) + ((1 - EMA_ALPHA) * speed_ema)

            now = time.time()
            if now - last_print >= 0.25:
                last_print = now

                good_fix = (fix_q >= 1) and (sats >= MIN_SATS) and (hdop <= MAX_HDOP)

                # Decide displayed speed
                if not good_fix or speed_ema is None:
                    disp = "--"
                else:
                    disp_speed = 0.0 if speed_ema < STOP_THRESHOLD_KMH else speed_ema
                    disp = f"{disp_speed:6.2f} km/h"

                fix_str = "NO FIX" if fix_q == 0 else f"FIX({fix_q})"
                print(f"{fix_str:8} | sats:{sats:2d} | hdop:{hdop:5.2f} | speed:{disp}")

    except KeyboardInterrupt:
        print("\nExited.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
