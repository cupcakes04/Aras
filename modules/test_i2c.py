
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import time

I2C_BUS = 7        # change if needed
OLED_ADDR = 0x3C   # 0x3C or 0x3D

def main():
    serial = i2c(
        port=I2C_BUS,
        address=OLED_ADDR
    )

    device = ssd1306(serial)

    img = Image.new("1", (device.width, device.height), 0)
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    draw.text((0, 0), "HELLO OLED", font=font, fill=255)
    draw.text((0, 16), "I2C TEST", font=font, fill=255)

    device.display(img)

    print("Text sent to OLED")
    time.sleep(10)

if __name__ == "__main__":
    main()
