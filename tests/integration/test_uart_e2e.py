# -*- coding: utf-8 -*-

"""End-to-end UART test.

`Blink_uart_m3` prints ``millis0``/``millis1`` around a pair of ``delay(5)``
calls, so a correct run proves the whole chain works:

    PMC clock -> SysTick -> NVIC interrupt -> firmware -> UART registers
    -> serial bridge -> an external program

It also exercises the interrupt engine: ``millis()`` only advances by 10 ms per
loop iteration if SysTick interrupts are actually delivered.

The test drives the portable TCP bridge, so it runs on Linux, macOS and Windows
without socat.
"""

import socket
import time

import pytest

from xuanwu import XuanWu

pytestmark = pytest.mark.integration

TIMEOUT = 60
CHUNK = 5_000_000


def tcp_peer(device) -> socket.socket:
    """Connect to the UART bridge's advertised peer endpoint."""
    hint = device.hw.perif["uart"].peer_hint
    assert hint.startswith("tcp://"), f"expected a TCP bridge, got {hint!r}"
    host, port = hint[len("tcp://") :].rsplit(":", 1)
    return socket.create_connection((host, int(port)), timeout=5)


def test_firmware_writes_to_the_virtual_serial_port(sam3x8e_path, sam3x8e_firmware):
    device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "tcp"})
    device.reset()

    captured = bytearray()
    with tcp_peer(device) as peer:
        peer.settimeout(0.2)
        deadline = time.monotonic() + TIMEOUT
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
    assert "millis0 =" in text, f"no serial output captured: {text!r}"

    values = [int(line.split("=")[1]) for line in text.splitlines() if line.startswith("millis1")]
    assert len(values) >= 2, f"expected at least two samples, got {values}"
    # delay(5)+delay(5) must advance the millisecond counter.
    assert values[-1] > values[0], f"millis() did not advance: {values}"
    assert values[1] - values[0] >= 10, f"each loop iteration should take >= 10 ms: {values}"


def test_uart_register_reads_reach_the_firmware_through_the_loopback_bridge(sam3x8e_path, sam3x8e_firmware):
    """Bytes pushed into the bridge must reach the simulated UART's receive path."""
    from xuanwu.arch.vendor.atmel.uart import UART_CR

    device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "loopback"})
    device.reset()

    uart = device.hw.perif["uart"]
    bridge = uart._bridge  # noqa: SLF001 - the bridge is the subject under test

    # Wait for the firmware's setup() to enable the receiver: until then the
    # model deliberately drops whatever arrives, exactly like real hardware.
    for _ in range(200):
        device.run(count=50_000)
        if uart.read_register("CR") & (1 << UART_CR.RXEN):
            break
    else:
        pytest.fail("the firmware never enabled the UART receiver")

    bridge.feed(b"Z")

    # The firmware polls the UART, so it consumes the byte itself; what proves the
    # path is that the received value ended up in the model's receive register.
    for _ in range(20):
        device.run(count=200_000)
        if bridge.in_waiting == 0:
            break

    assert bridge.in_waiting == 0, "the firmware never consumed the byte from the bridge"
    assert uart._last_rx == ord("Z"), "the received byte did not reach the UART model"  # noqa: SLF001
