# -*- coding: utf-8 -*-

"""An Arduino Due with the Adafruit TFT shield, running Adafruit's own library.

The shield is two SPI devices on one controller (an ILI9341 on D10, a microSD socket
on D4), so the chip description declares the panel on a shared bus and the SPI model
routes each byte to whoever its chip select has selected.  The firmware is built from
`tests/firmware/Tft_smoke_m3/` against the library versions pinned in the Dockerfile.

    python examples/tft.py                 # runs the sketch, writes tft.png
    python examples/tft.py out.png         # ... somewhere else

Everything here needs no display: the panel's frame buffer is just memory, and the PNG
is written by xuanwu itself (`DisplaySurface.to_png`), not by an image library.
"""

import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402
from xuanwu.peripherals.display import rgb565  # noqa: E402

CHIP = "sam3x8e_tft"
FIRMWARE = REPO_ROOT / "tests/firmware/due_tft_smoke/Tft_smoke_m3.ino.elf"
CHUNK = 5_000_000


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    output = Path(argv[0]) if argv else REPO_ROOT / "tft.png"

    logging.disable(logging.CRITICAL)
    device = XuanWu(CHIP, str(FIRMWARE), hardware_options={"bridge": "loopback"})
    device.reset()
    uart = device.hw.perif["uart"]._bridge  # noqa: SLF001 - the loopback bridge is ours
    panel = device.dev["LCD"]

    text = b""
    for _ in range(40):
        device.run(count=CHUNK)
        text += uart.drain()
        if b"smoke done" in text:
            break
    logging.disable(logging.NOTSET)

    print("the sketch printed:")
    for line in text.decode("utf-8", "replace").splitlines():
        print(f"  {line}")

    print("\nthe panel received:")
    print(f"  {panel.bytes_received} bytes, {len(panel.core.commands)} commands")
    print(f"  first commands: {[hex(c) for c in panel.core.commands[:8]]}")
    print(f"  display on: {panel.core.display_on}, MADCTL: 0x{panel.core.madctl:02x}")

    print("\nwhat the host sees in the frame buffer:")
    red, green, blue, white = (rgb565(255, 0, 0), rgb565(0, 255, 0), rgb565(0, 0, 255), 0xFFFF)
    print(f"  size          {panel.size}")
    print(f"  red  fillRect {panel.pixel(10, 20):#06x} at (10, 20)   ({panel.surface.count(red)} px)")
    print(f"  green frame   {panel.pixel(50, 60):#06x} at (50, 60)   ({panel.surface.count(green)} px)")
    print(f"  blue  hline   {panel.pixel(120, 0):#06x} at (120, 0)   ({panel.surface.count(blue)} px)")
    print(f"  white pixel   {panel.pixel(200, 300):#06x} at (200, 300)")
    print(f"  frame digest  {panel.surface.digest()[:16]}...")

    path = panel.save(str(output))
    print(f"\nwrote {path} ({Path(path).stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
