# -*- coding: utf-8 -*-

"""Read what a firmware prints on its UART, from another process.

The simulated UART is exposed to the host by a *bridge*; the default on POSIX is a
``socat`` pty pair, and ``tcp`` makes the simulator listen on a port that any
program (or another machine) can connect to.  This example uses the portable TCP
bridge, so it behaves the same on Linux, macOS and Windows.

    python examples/uart_bridge.py

It runs the bundled SAM3X8E firmware, whose ``loop()`` prints ``millis0`` and
``millis1`` around two ``delay(5)`` calls, and prints the lines it captures.
"""

import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402

CHIP = "sam3x8e"
FIRMWARE = REPO_ROOT / "tests/firmware/sam3x8e/Blink_uart_m3.ino.elf"

CHUNK = 2_000_000  # instructions per slice, so the socket can be drained in between
DEADLINE = 30.0  # seconds of wall-clock time


def main() -> int:
    device = XuanWu(CHIP, str(FIRMWARE), hardware_options={"bridge": "tcp"})
    device.reset()

    hint = device.hw.perif["uart"].peer_hint  # tcp://127.0.0.1:<port>
    print(f"UART bridge : {hint}")
    host, port = hint[len("tcp://") :].rsplit(":", 1)

    captured = bytearray()
    with socket.create_connection((host, int(port)), timeout=5) as peer:
        peer.settimeout(0.2)
        deadline = time.monotonic() + DEADLINE
        while time.monotonic() < deadline and captured.count(b"millis1") < 2:
            device.run(count=CHUNK)
            try:
                while True:
                    chunk = peer.recv(4096)
                    if not chunk:
                        break
                    captured.extend(chunk)
            except socket.timeout:
                pass

    text = captured.decode("utf-8", "replace")
    print("--- captured from the simulated serial port ---")
    print(text.strip() or "(nothing: the firmware never wrote)")

    values = [int(line.split("=")[1]) for line in text.splitlines() if line.startswith("millis1")]
    if len(values) < 2:
        print(f"expected two millis1 samples, got {values}")
        return 1
    print(f"--- millis() advanced by {values[1] - values[0]} ms across two delay(5) calls ---")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
