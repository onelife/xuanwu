# -*- coding: utf-8 -*-

"""Unit tests for the serial bridges."""

import os
import socket
import time

import pytest

from xuanwu.backends import BRIDGE_KINDS, LoopbackBridge, TcpBridge, create_bridge
from xuanwu.exception import XwSerialBridgeError


class TestLoopbackBridge:
    def test_starts_empty(self):
        bridge = LoopbackBridge()
        assert bridge.in_waiting == 0
        assert bridge.read(4) == b""
        assert bridge.drain() == b""

    def test_device_reads_what_the_test_feeds(self):
        bridge = LoopbackBridge()
        bridge.feed(b"hello")
        assert bridge.in_waiting == 5
        assert bridge.read(2) == b"he"
        assert bridge.read(10) == b"llo"
        assert bridge.in_waiting == 0

    def test_device_writes_are_drained(self):
        bridge = LoopbackBridge()
        assert bridge.write(b"\x01\x02") == 2
        assert bridge.drain() == b"\x01\x02"
        assert bridge.drain() == b""

    def test_reset_buffers(self):
        bridge = LoopbackBridge()
        bridge.feed(b"abc")
        bridge.write(b"xyz")
        bridge.reset_input_buffer()
        bridge.reset_output_buffer()
        assert bridge.in_waiting == 0
        assert bridge.drain() == b""

    def test_uses_no_host_resources(self):
        bridge = LoopbackBridge()
        assert bridge.peer_hint == "loopback"
        bridge.close()


class TestTcpBridge:
    def test_peer_can_connect_after_construction(self):
        bridge = TcpBridge()
        try:
            assert bridge.peer_hint.startswith("tcp://127.0.0.1:")
            port = int(bridge.peer_hint.rsplit(":", 1)[1])
            with socket.create_connection(("127.0.0.1", port), timeout=5) as peer:
                # bytes written before the peer connected must not be lost
                bridge.write(b"early")
                assert _recv_at_least(peer, 5) == b"early"

                peer.sendall(b"ping")
                assert _wait_for(lambda: bridge.in_waiting >= 4)
                assert bridge.read(4) == b"ping"
        finally:
            bridge.close()

    def test_close_is_idempotent(self):
        bridge = TcpBridge()
        bridge.close()
        bridge.close()


class TestFactory:
    def test_explicit_kinds(self):
        assert isinstance(create_bridge("loopback"), LoopbackBridge)
        assert isinstance(create_bridge("tcp"), TcpBridge)
        for bridge in (create_bridge("loopback"), create_bridge("tcp")):
            bridge.close()

    def test_unknown_kind_is_rejected(self):
        with pytest.raises(XwSerialBridgeError) as excinfo:
            create_bridge("carrier-pigeon")
        assert "carrier-pigeon" in str(excinfo.value)

    def test_auto_picks_a_usable_bridge(self):
        bridge = create_bridge("auto")
        try:
            assert bridge.kind in BRIDGE_KINDS
            assert bridge.peer_hint
        finally:
            bridge.close()

    def test_auto_uses_socat_when_available(self, monkeypatch):
        created = {}

        class StubSocat(LoopbackBridge):
            kind = "socat"

            def __init__(self, **kwargs):
                created.update(kwargs)
                super().__init__(**kwargs)

        monkeypatch.setattr("xuanwu.backends.serial_bridge.shutil.which", lambda name: "/usr/bin/socat")
        monkeypatch.setitem(BRIDGE_KINDS, "socat", StubSocat)

        bridge = create_bridge("auto", prefix="uart", baudrate=9600)
        try:
            assert isinstance(bridge, StubSocat)
            assert created["prefix"] == "uart"
            assert created["baudrate"] == 9600
        finally:
            bridge.close()

    def test_auto_falls_back_to_tcp_without_socat(self, monkeypatch):
        """This is what makes sam3x8e.yaml usable on Windows."""
        monkeypatch.setattr("xuanwu.backends.serial_bridge.shutil.which", lambda name: None)
        bridge = create_bridge("auto")
        try:
            assert isinstance(bridge, TcpBridge)
        finally:
            bridge.close()


class TestSocatBridge:
    @pytest.mark.skipif(not __import__("shutil").which("socat"), reason="socat is not installed")
    def test_creates_a_pty_pair(self):
        from xuanwu.backends import SocatBridge

        bridge = SocatBridge(prefix="xwtest")
        try:
            assert os.path.islink(bridge.peer_hint)
            assert os.path.islink(bridge.device_port)
            assert bridge.device_port != bridge.peer_hint
        finally:
            peer = bridge.peer_hint
            bridge.close()
            assert not os.path.lexists(peer)

    @pytest.mark.skipif(not __import__("shutil").which("socat"), reason="socat is not installed")
    def test_data_flows_both_ways_through_the_pty_pair(self):
        from serial import Serial

        from xuanwu.backends import SocatBridge

        bridge = SocatBridge(prefix="xwtest")
        try:
            with Serial(bridge.peer_hint, 115200, timeout=1) as peer:
                bridge.write(b"hello")
                assert peer.read(5) == b"hello"

                peer.write(b"world")
                assert _wait_for(lambda: bridge.in_waiting >= 5)
                assert bridge.read(5) == b"world"
        finally:
            bridge.close()


def _recv_at_least(sock: socket.socket, size: int, timeout: float = 5.0) -> bytes:
    sock.settimeout(timeout)
    buf = b""
    deadline = time.monotonic() + timeout
    while len(buf) < size and time.monotonic() < deadline:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf


def _wait_for(predicate, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()
