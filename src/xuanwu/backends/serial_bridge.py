# -*- coding: utf-8 -*-

"""Host serial bridges.

A peripheral that talks to the outside world needs two ends of a byte stream: one
that the simulated device reads and writes, and one an external program attaches
to.  Three implementations are provided:

* :class:`SocatBridge` -- a POSIX pty pair created by ``socat`` (the original
  behaviour, kept for Linux/macOS).
* :class:`TcpBridge` -- the simulator listens on a TCP port and the external
  program connects to it.  Works everywhere, Windows included, and needs no
  extra packages.
* :class:`LoopbackBridge` -- an in-memory duplex buffer for tests.  Uses no host
  resources at all.

``create_bridge("auto")`` prefers ``socat`` when it is installed and falls back
to TCP otherwise, which is what the chip YAML asks for by default.
"""

import os
import shutil
import socket
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from ..config import logger
from ..exception import XwSerialBridgeError

__all__ = [
    "SerialBridge",
    "SocatBridge",
    "TcpBridge",
    "LoopbackBridge",
    "create_bridge",
    "BRIDGE_KINDS",
]


class SerialBridge(ABC):
    """One end of a bidirectional byte stream."""

    kind = "abstract"

    @property
    @abstractmethod
    def peer_hint(self) -> str:
        """How an external program should attach to the other end."""

    @property
    @abstractmethod
    def in_waiting(self) -> int:
        """Number of bytes available to :meth:`read` right now."""

    @abstractmethod
    def read(self, size: int = 1) -> bytes:
        """Return up to ``size`` bytes; may return fewer (or nothing)."""

    @abstractmethod
    def write(self, data: bytes) -> int:
        """Send ``data`` to the peer."""

    def reset_input_buffer(self) -> None:
        """Discard anything the peer sent that has not been read yet."""

    def reset_output_buffer(self) -> None:
        """Discard anything queued for the peer."""

    def close(self) -> None:
        """Release every host resource; safe to call more than once."""


class SocatBridge(SerialBridge):
    """A pty pair wired together by ``socat`` (POSIX only)."""

    kind = "socat"
    COMMAND = "socat -d -d pty,link={device},raw,echo=0 pty,link={peer},raw,echo=0"

    def __init__(self, baudrate: int = 115200, prefix: str = "tty", timeout: float = 3.0, interval: float = 0.02):
        from tempfile import mkstemp

        self._proc: Optional[Any] = None
        self._serial: Optional[Any] = None
        self._device = ""
        self._peer = ""

        device_fd, device_path = mkstemp(prefix=f"{prefix}_")
        peer_fd, peer_path = mkstemp(prefix=f"{prefix}_peer_")
        os.close(device_fd)
        os.close(peer_fd)
        self._device = device_path
        self._peer = peer_path

        try:
            self._proc = self._spawn(timeout, interval)
            from serial import Serial

            self._serial = Serial(self._device, baudrate)
        except BaseException:
            self.close()
            raise

    def _spawn(self, timeout: float, interval: float):
        """Start socat and wait until it has replaced the temp files with pty links."""
        import shlex
        from subprocess import PIPE, Popen

        command = self.COMMAND.format(device=self._device, peer=self._peer)
        try:
            proc = Popen(shlex.split(command), stdout=PIPE, stderr=PIPE)
        except FileNotFoundError as err:
            raise XwSerialBridgeError(
                "'socat' was not found on PATH. Install it (Debian/Ubuntu: 'apt-get install socat') "
                "or select another bridge, e.g. 'bridge: tcp'."
            ) from err

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.islink(self._device) and os.path.exists(self._device):
                return proc
            if proc.poll() is not None:
                _, err = proc.communicate()
                raise XwSerialBridgeError(
                    f"socat exited with code {proc.returncode}: {err.decode('utf-8', 'replace').strip()}"
                )
            time.sleep(interval)

        proc.kill()
        proc.communicate()
        raise XwSerialBridgeError(f"timed out after {timeout:g}s waiting for the pty link {self._device}")

    @property
    def peer_hint(self) -> str:
        return self._peer

    @property
    def device_port(self) -> str:
        return self._device

    @property
    def in_waiting(self) -> int:
        return self._serial.in_waiting if self._serial is not None else 0

    def read(self, size: int = 1) -> bytes:
        return self._serial.read(size) if self._serial is not None else b""

    def write(self, data: bytes) -> int:
        return self._serial.write(data) if self._serial is not None else 0

    def reset_input_buffer(self) -> None:
        if self._serial is not None:
            self._serial.reset_input_buffer()

    def reset_output_buffer(self) -> None:
        if self._serial is not None:
            self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - destructor path
                pass
            self._serial = None
        proc, self._proc = self._proc, None
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
                proc.wait(timeout=1)
            except Exception:  # noqa: BLE001 - destructor path
                pass
        for target in (self._device, self._peer):
            if target and os.path.lexists(target):
                try:
                    os.remove(target)
                except OSError:
                    pass


class TcpBridge(SerialBridge):
    """The simulator listens on a TCP port; the peer connects to it.

    This is the portable option: no pty, no extra process, no driver.  Bytes
    written before a peer connects are buffered (up to ``max_pending``) so that
    short runs are not lost.
    """

    kind = "tcp"

    def __init__(
        self,
        baudrate: int = 115200,
        host: str = "127.0.0.1",
        port: int = 0,
        max_pending: int = 256 * 1024,
        **_kwargs: Any,
    ):
        self._max_pending = max_pending
        self._lock = threading.Lock()
        self._rx = bytearray()
        self._tx = bytearray()
        self._conn: Optional[socket.socket] = None
        self._closed = False

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(1)
        self._host, self._port = self._server.getsockname()

        self._thread = threading.Thread(target=self._serve, name=f"xuanwu-{host}:{self._port}", daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        try:
            while not self._closed:
                conn, addr = self._server.accept()
                logger.info(f"Serial bridge peer connected: {addr[0]}:{addr[1]}")
                with conn:
                    with self._lock:
                        self._conn = conn
                        pending, self._tx = self._tx, bytearray()
                    if pending:
                        conn.sendall(bytes(pending))
                    while not self._closed:
                        try:
                            chunk = conn.recv(4096)
                        except OSError:
                            break
                        if not chunk:
                            break
                        with self._lock:
                            self._rx.extend(chunk)
                    with self._lock:
                        self._conn = None
        except OSError as err:
            if not self._closed:
                logger.debug(f"Serial bridge accept loop ended: {err}")

    @property
    def peer_hint(self) -> str:
        return f"tcp://{self._host}:{self._port}"

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            chunk = bytes(self._rx[:size])
            del self._rx[:size]
        return chunk

    def write(self, data: bytes) -> int:
        with self._lock:
            conn = self._conn
            if conn is None:
                self._tx.extend(data)
                if len(self._tx) > self._max_pending:
                    del self._tx[: len(self._tx) - self._max_pending]
                return len(data)
        try:
            conn.sendall(data)
        except OSError as err:
            logger.debug(f"Serial bridge write failed: {err}")
        return len(data)

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def reset_output_buffer(self) -> None:
        with self._lock:
            self._tx.clear()

    def close(self) -> None:
        self._closed = True
        with self._lock:
            conn, self._conn = self._conn, None
        for sock in (conn, getattr(self, "_server", None)):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=1)


class LoopbackBridge(SerialBridge):
    """An in-memory duplex buffer, for tests that need no host resources."""

    kind = "loopback"

    def __init__(self, baudrate: int = 115200, **_kwargs: Any):
        self._lock = threading.Lock()
        self._rx = bytearray()  # fed by the test, read by the device
        self._tx = bytearray()  # written by the device, drained by the test

    # -- test helpers ------------------------------------------------
    def feed(self, data: bytes) -> int:
        """Queue bytes for the simulated device to receive."""
        with self._lock:
            self._rx.extend(data)
        return len(data)

    def drain(self) -> bytes:
        """Take everything the simulated device has transmitted."""
        with self._lock:
            out = bytes(self._tx)
            self._tx.clear()
        return out

    # -- SerialBridge ------------------------------------------------
    @property
    def peer_hint(self) -> str:
        return "loopback"

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            chunk = bytes(self._rx[:size])
            del self._rx[:size]
        return chunk

    def write(self, data: bytes) -> int:
        with self._lock:
            self._tx.extend(data)
        return len(data)

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def reset_output_buffer(self) -> None:
        with self._lock:
            self._tx.clear()

    def close(self) -> None:
        self.reset_input_buffer()
        self.reset_output_buffer()


BRIDGE_KINDS: Dict[str, Any] = {
    "socat": SocatBridge,
    "tcp": TcpBridge,
    "loopback": LoopbackBridge,
}
"""Bridges that can be requested explicitly from the chip YAML."""


def create_bridge(kind: str = "auto", **kwargs: Any) -> SerialBridge:
    """Instantiate a bridge.

    ``"auto"`` uses ``socat`` when it is available and falls back to TCP, so the
    same chip description works on Linux, macOS and Windows.
    """
    kind = (kind or "auto").lower()
    if kind == "auto":
        kind = "socat" if shutil.which("socat") else "tcp"
    if kind not in BRIDGE_KINDS:
        raise XwSerialBridgeError(f"Unknown serial bridge {kind!r}; choose from {sorted(BRIDGE_KINDS)} or 'auto'")
    bridge = BRIDGE_KINDS[kind](**kwargs)
    logger.debug(f"Serial bridge {kind}: peer at {bridge.peer_hint}")
    return bridge
