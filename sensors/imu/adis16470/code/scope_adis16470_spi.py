#!/usr/bin/env python3

import argparse
import time
import spidev


def make_pattern(num_bytes):
    data = []

    for i in range(num_bytes):
        if i % 2 == 0:
            data.append(0xAA)   # 10101010
        else:
            data.append(0x55)   # 01010101

    return data


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--mode", type=int, default=3)
    parser.add_argument("--speed", type=int, default=10000)
    parser.add_argument("--bytes", type=int, default=256)
    parser.add_argument("--gap", type=float, default=0.001)

    args = parser.parse_args()

    spi = spidev.SpiDev()
    spi.open(args.bus, args.device)

    spi.mode = args.mode
    spi.max_speed_hz = args.speed
    spi.bits_per_word = 8
    spi.lsbfirst = False

    pattern = make_pattern(args.bytes)

    print()
    print("Steady SPI waveform test")
    print(f"bus={args.bus}, device={args.device}")
    print(f"mode={args.mode}, speed={args.speed} Hz")
    print(f"bytes per burst={args.bytes}, gap={args.gap} s")
    print()
    print("Probe:")
    print("  SCLK -> Pi physical pin 23")
    print("  MOSI -> Pi physical pin 19")
    print("  MISO -> Pi physical pin 21")
    print("  CE0  -> Pi physical pin 24")
    print("  GND  -> Pi GND")
    print()
    print("Press CTRL+C to stop.")
    print()

    count = 0
    last_print = time.monotonic()

    try:
        while True:
            rx = spi.xfer2(pattern)
            count += 1

            now = time.monotonic()

            if now - last_print >= 1.0:
                last_print = now
                print(
                    f"running... bursts={count} | "
                    f"first DOUT bytes: {[hex(x) for x in rx[:4]]}"
                )

            if args.gap > 0:
                time.sleep(args.gap)

    except KeyboardInterrupt:
        print()
        print("Stopped.")

    finally:
        spi.close()


if __name__ == "__main__":
    main()
