#!/usr/bin/env python3

import time
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)

spi.mode = 0b11
spi.max_speed_hz = 10000
spi.bits_per_word = 8
spi.lsbfirst = False

pattern = [0xAA, 0x55, 0x00, 0xFF]

print("Pi SPI loopback test")
print("Jumper Pi pin 19 MOSI to Pi pin 21 MISO")
print("Press CTRL+C to stop")
print()

try:
    while True:
        rx = spi.xfer2(pattern)

        print(
            "sent:",
            [f"0x{x:02X}" for x in pattern],
            "read:",
            [f"0x{x:02X}" for x in rx],
        )

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    spi.close()

