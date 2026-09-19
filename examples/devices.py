# -*- coding: utf-8 -*-

"""The external devices a chip description declares.

``sam3x8e.yaml`` wires an LED to ``GPIOB.27`` and a SPI NOR flash to the SPI port,
with ``GPIOC.26`` as its chip select.  Those devices are built when the simulation
starts and can be driven from Python through the same registers firmware would use.

    python examples/devices.py
"""

import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402

CHIP = "sam3x8e"
FIRMWARE = REPO_ROOT / "tests/firmware/sam3x8e/Blink_uart_m3.ino.elf"

GPIOB_BASE = 0x400E1000
GPIOC_BASE = 0x400E1200
PIO_SODR = 0x30  # set output data -> pin high
PIO_CODR = 0x34  # clear output data -> pin low

SPI_CR = 0x4000_8000
SPI_RDR = 0x4000_8008
SPI_TDR = 0x4000_800C

LED_PIN = 27
CS_PIN = 26

JEDEC_ID = 0x9F
WRITE_ENABLE = 0x06
PAGE_PROGRAM = 0x02
READ_DATA = 0x03


def main() -> int:
    logging.disable(logging.CRITICAL)
    # The SPI bus is driven in-process by the flash, and the UART has no host end
    # here, so nothing is opened on the host side.
    device = XuanWu(CHIP, str(FIRMWARE), hardware_options={"bridge": "none"})
    device.reset()

    print("devices declared by the chip description:")
    for name in device.dev.names():
        print(f"  {name}")

    # --- the LED --------------------------------------------------------
    led = device.dev["LED"]
    gpiob = device.hw.perif["gpiob"]
    led.reset()
    gpiob.write(GPIOB_BASE + PIO_SODR, 4, 1 << LED_PIN)  # drive the pin high
    print(f"\nGPIOB.{LED_PIN} high -> LED lit: {led.state}, transitions: {led.transitions}")
    gpiob.write(GPIOB_BASE + PIO_CODR, 4, 1 << LED_PIN)
    print(f"GPIOB.{LED_PIN} low  -> LED lit: {led.state}, transitions: {led.transitions}")

    # --- the SPI flash --------------------------------------------------
    spi = device.hw.perif["spi"]
    gpioc = device.hw.perif["gpioc"]

    def cs(low: bool) -> None:
        gpioc.write(GPIOC_BASE + (PIO_CODR if low else PIO_SODR), 4, 1 << CS_PIN)

    def send(values) -> None:
        for value in values:
            spi.write(SPI_TDR, 4, value)

    def receive(count: int) -> bytes:
        return bytes(spi.read(SPI_RDR, 4) & 0xFF for _ in range(count))

    flash = device.dev["FLASH"]
    flash.reset()

    cs(True)
    send([JEDEC_ID])  # "who are you?"
    print(f"\nflash JEDEC ID: {receive(3).hex()} (Winbond W25Q128: ef4018)")
    cs(False)
    print(f"flash size: 0x{flash.size:x} bytes, first byte: 0x{flash.read_memory(0, 1)[0]:02x}")

    cs(True)
    send([WRITE_ENABLE])
    cs(False)

    cs(True)
    send([PAGE_PROGRAM, 0x00, 0x00, 0x40])  # program at 0x40
    send(b"xuanwu")
    cs(False)
    print(f"after page program, flash[0x40:0x46] = {flash.read_memory(0x40, 6)!r}")

    cs(True)
    send([READ_DATA, 0x00, 0x00, 0x40])
    send(b"\x00" * 6)  # clock the bytes out
    print(f"read back over the bus:            {receive(6)!r}")
    cs(False)

    if flash.read_memory(0x40, 6) != b"xuanwu":
        print("the write did not reach the flash")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
