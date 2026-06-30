#!/usr/bin/env python3

import time
import spidev


def open_spi(mode, speed):
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.mode = mode
    spi.max_speed_hz = speed
    spi.bits_per_word = 8
    spi.lsbfirst = False
    spi.no_cs = False
    time.sleep(1.0)
    return spi


def transfer(spi, tx):
    tx_saved = list(tx)
    rx = spi.xfer2(list(tx))
    return tx_saved, rx


def show(label, tx, rx):
    tx_text = " ".join(f"{b:02X}" for b in tx)
    rx_text = " ".join(f"{b:02X}" for b in rx)
    print(f"{label:<12} TX: {tx_text:<15} RX: {rx_text}")


def test(mode, speed):
    print()
    print(f"Testing mode={mode}, speed={speed}")
    print("-" * 50)

    spi = open_spi(mode, speed)

    try:
        for i in range(10):
            print(f"Cycle {i}")

            tx, rx = transfer(spi, [0x72, 0x00])
            show("cmd 7200", tx, rx)

            time.sleep(0.002)

            tx, rx = transfer(spi, [0x00, 0x00])
            show("dummy1", tx, rx)

            time.sleep(0.002)

            tx, rx = transfer(spi, [0x00, 0x00])
            show("dummy2", tx, rx)

            time.sleep(0.002)

            tx, rx = transfer(spi, [0x72, 0x00, 0x00, 0x00])
            show("32 clock", tx, rx)

            print()
            time.sleep(0.05)

    finally:
        spi.close()


def main():
    test(mode=0, speed=1000000)
    test(mode=3, speed=1000000)
    test(mode=3, speed=100000)


if __name__ == "__main__":
    main()
