# -*- coding: utf-8 -*-

"""Integration tests for the GDB stub.

These cover the two defects that used to kill the whole stub:
  * a single-register read ('p') returned raw bytes into a str-only encoder
  * any packet the stub does not implement returned None into the encoder
"""

import socket
import threading
import time

import pytest

from xuanwu.rsp import RemoteSerialProtocol

pytestmark = pytest.mark.integration

# Scratch RAM in the STM32F411 memory map (top of the 128 KiB SRAM).
SCRATCH = 0x2001F000


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def send(sock: socket.socket, payload: str) -> None:
    body = payload.encode()
    sock.sendall(b"$" + body + b"#" + b"%02x" % (sum(body) & 0xFF))


def recv_packet(sock: socket.socket, timeout: float = 3.0) -> bytes:
    """Read one reply; the reply may be prefixed by a '+' ack character."""
    sock.settimeout(timeout)
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if b"#" in buf:
            break
    return buf


def body_of(reply: bytes) -> bytes:
    """Decode a reply body, tolerating leading ack characters and RLE encoding."""
    decoded = RemoteSerialProtocol.read_data(None, reply)
    return b"" if decoded is None else decoded


@pytest.fixture(scope="module")
def stub(stm32f411_path, stm32f411_firmware):
    """Run a GDB stub for the STM32F411 firmware on a private port."""
    from xuanwu import XuanWu

    port = free_port()
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware), rsp=port)
    device.reset()
    errors = []

    def serve():
        try:
            device.run()
        except BaseException as err:  # noqa: BLE001 - recorded and asserted on
            errors.append(err)

    threading.Thread(target=serve, daemon=True).start()

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            connection = socket.create_connection(("127.0.0.1", port), timeout=1)
            break
        except OSError:
            time.sleep(0.05)
    else:
        pytest.fail("GDB stub did not start listening")

    yield connection, errors, device

    connection.close()


class TestStubSurvivesUnsupportedPackets:
    def test_qsupported_advertises_target_description(self, stub):
        connection, _errors, _device = stub
        send(connection, "qSupported:swbreak+;hwbreak+")
        assert b"qXfer:features:read+" in recv_packet(connection)

    def test_qxfer_serves_the_target_description(self, stub):
        connection, _errors, _device = stub
        send(connection, "qXfer:features:read:target.xml:0,fff")
        body = body_of(recv_packet(connection))
        assert body[:1] in (b"l", b"m")
        assert b"<?xml" in body
        # GDB ignores a bare <feature> document and silently falls back to its
        # default register set, which then fails with
        # "Truncated register 16 in remote 'g' packet".
        assert b"<target>" in body
        assert b"<architecture>arm</architecture>" in body

    def test_g_packet_matches_the_target_description(self, stub):
        connection, _errors, _device = stub
        send(connection, "g")
        body = body_of(recv_packet(connection))
        # 17 registers (r0-r12, sp, lr, pc, xpsr) x 4 bytes x 2 hex chars
        assert len(body) == 17 * 4 * 2
        assert all(c in b"0123456789abcdef" for c in body)

    def test_qxfer_for_an_unknown_object_reports_unsupported(self, stub):
        connection, _errors, _device = stub
        send(connection, "qXfer:memory-map:read::0,fff")
        assert body_of(recv_packet(connection)) == b"E00"

    def test_single_register_read_is_answered(self, stub):
        # Used to raise TypeError inside write_data and kill the stub.
        connection, errors, _device = stub
        send(connection, "p0")
        body = body_of(recv_packet(connection))
        assert len(body) == 8 and all(c in b"0123456789abcdef" for c in body)
        assert errors == []

    def test_unimplemented_packet_gets_empty_reply(self, stub):
        connection, errors, _device = stub
        send(connection, "M0,4:deadbeef")
        assert recv_packet(connection).startswith((b"$", b"+$"))
        assert errors == []

    def test_stub_still_works_afterwards(self, stub):
        connection, errors, _device = stub
        send(connection, "?")
        assert body_of(recv_packet(connection)).startswith(b"T")
        assert errors == []


class TestStubControl:
    def test_single_step_advances_pc(self, stub):
        connection, errors, device = stub
        before = device.reg.pc
        send(connection, "s")
        assert body_of(recv_packet(connection)) == b"S05"
        assert device.reg.pc != before
        assert errors == []

    def test_breakpoint_is_hit(self, stub):
        connection, errors, device = stub
        # Step forward first so the target is guaranteed to be a real
        # instruction boundary reachable by linear execution.
        start = device.reg.pc
        reached = []
        for _ in range(4):
            send(connection, "s")
            recv_packet(connection)
            reached.append(device.reg.pc)
        target = reached[-1]
        device.reg.write("pc", start)

        send(connection, "Z0,%x,2" % target)
        assert body_of(recv_packet(connection)) == b"OK"
        send(connection, "c")
        assert body_of(recv_packet(connection)) == b"S05"
        assert device.reg.pc == target

        send(connection, "z0,%x,2" % target)
        assert body_of(recv_packet(connection)) == b"OK"
        assert errors == []

    def test_memory_write_round_trip(self, stub):
        connection, _errors, _device = stub
        send(connection, "X%08x,4:\x01\x02\x03\x04" % SCRATCH)
        assert body_of(recv_packet(connection)) == b"OK"
        send(connection, "m%08x,4" % SCRATCH)
        assert body_of(recv_packet(connection)) == b"01020304"
