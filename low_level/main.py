from periphery import Serial
import time

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# ---- EDIT THESE ----
GPS_DEV = "/dev/ttyAS4"   # change to /dev/ttyAS0 if you move wiring (avoid ttyAS0 if it's console)
GPS_BAUD = 9600

I2C_BUS = 7               # set to the /dev/i2c-X number that detects your OLED
OLED_ADDR = 0x3C          # most common is 0x3C; use 0x3D if your scan shows 3d
# --------------------

KNOTS_TO_KMH = 1.852

# Tuning knobs
MIN_SATS = 6
MAX_HDOP = 3.0
STOP_THRESHOLD_KMH = 2.0   # treat anything below this as 0 when stationary (raise if indoors)
EMA_ALPHA = 0.2            # lower = smoother, slower response


def read_cpu_temp_c():
    """Read CPU temperature from sysfs. Returns float degC or None."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            raw = f.read().strip()
        return int(raw) / 1000.0
    except Exception:
        return None


def parse_gga(line: str):
    # $GNGGA,time,lat,N,lon,E,fix,numsats,hdop,alt,M,...
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
    # VTG preferred (km/h)
    if line.startswith(("$GNVTG", "$GPVTG")):
        parts = line.split(",")
        # Standard VTG: ..., speed_knots, N, speed_kmh, K*CS
        if len(parts) > 9 and parts[7]:
            try:
                return float(parts[7])
            except ValueError:
                return None

    # RMC fallback (knots)
    if line.startswith(("$GNRMC", "$GPRMC")):
        parts = line.split(",")
        if len(parts) > 7:
            status = parts[2].strip()  # A=valid, V=void
            if status == "A" and parts[7]:
                try:
                    return float(parts[7]) * KNOTS_TO_KMH
                except ValueError:
                    return None

    return None


def draw_screen(device, fix_q, sats, hdop, speed_kmh_display, good_fix, cpu_temp_c):
    img = Image.new("1", (device.width, device.height), 0)
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    # Line 1: GPS fix (left) + CPU temp (right)
    left = "GPS: NO FIX" if fix_q == 0 else f"GPS: FIX({fix_q})"
    right = "--.-C" if cpu_temp_c is None else f"{cpu_temp_c:4.1f}C"

    draw.text((0, 0), left, font=font, fill=255)
    right_w = draw.textlength(right, font=font)
    draw.text((device.width - right_w, 0), right, font=font, fill=255)

    # Line 2: sats + hdop
    draw.text((0, 14), f"Sats:{sats:2d} HDOP:{hdop:4.2f}", font=font, fill=255)

    # Line 3-4: speed
    if not good_fix:
        draw.text((0, 30), "Speed: --", font=font, fill=255)
        draw.text((0, 44), "km/h", font=font, fill=255)
    else:
        draw.text((0, 30), f"Speed: {speed_kmh_display:5.2f}", font=font, fill=255)
        draw.text((0, 44), "km/h", font=font, fill=255)

    device.display(img)


def main():
    # OLED init
    oled_bus = i2c(port=I2C_BUS, address=OLED_ADDR)
    device = ssd1306(oled_bus)

    # GPS init
    ser = Serial(GPS_DEV, GPS_BAUD)
    buf = bytearray()

    fix_q, sats, hdop = 0, 0, 99.99
    speed_ema = None

    last_draw = 0.0

    print(f"OLED: SSD1306 @ i2c-{I2C_BUS} addr {hex(OLED_ADDR)}")
    print(f"GPS:  {GPS_DEV} @ {GPS_BAUD}")
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

                    # Ignore noisy TXT spam
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
                    speed_disp = 0.0  # placeholder; display will show "--" if not good_fix

                draw_screen(device, fix_q, sats, hdop, speed_disp, good_fix, cpu_temp_c)

    except KeyboardInterrupt:
        print("\nExited.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
