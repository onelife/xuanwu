# -*- coding: utf-8 -*-

"""Arm semihosting.

Firmware built with the semihosting spec files (``--specs=rdimon.specs`` or
``--specs=rdimon-v2m.specs``) asks the host to do things by putting an operation
number in ``r0``, a parameter in ``r1`` and executing ``BKPT 0xAB``.  This is how
a bare-metal program gets ``printf`` to appear on your terminal without any UART
model in the way.

Unicorn implements QEMU's ARM semihosting but does not expose the switch that
enables it, so it reports the instruction as an ordinary ``BKPT``.
:class:`~xuanwu.arch.cortex_m.controller.ArmHardwareController` therefore
recognises the trap by its immediate, calls :meth:`SemiHosting.handle` and steps
over the instruction.  A plain ``BKPT`` still behaves as a breakpoint.

The console operations are implemented; file operations report failure, because
nothing in the simulator needs them yet.  Unsupported calls are logged and return
``-1`` rather than raising.
"""

import sys
import time
from enum import IntEnum
from typing import Any, Dict, List, Optional, TextIO

from ..config import logger

__all__ = ["SemiHosting", "SemiHostingOp", "SEMIHOST_BKPT"]


SEMIHOST_BKPT = 0xBEAB
"""Thumb encoding of ``BKPT 0xAB``, the semihosting trap."""

ADP_STOPPED_APPLICATION_EXIT = 0x20026


class SemiHostingOp(IntEnum):
    """Operation numbers, from the Arm semihosting specification."""

    OPEN = 0x01
    CLOSE = 0x02
    WRITEC = 0x03
    WRITE0 = 0x04
    WRITE = 0x05
    READ = 0x06
    READC = 0x07
    ISERROR = 0x08
    ISTTY = 0x09
    SEEK = 0x0A
    FLEN = 0x0C
    TMPNAM = 0x0D
    REMOVE = 0x0E
    RENAME = 0x0F
    CLOCK = 0x10
    TIME = 0x11
    SYSTEM = 0x12
    ERRNO = 0x13
    GET_CMDLINE = 0x15
    HEAPINFO = 0x16
    EXIT = 0x18
    EXIT_EXTENDED = 0x20
    ELAPSED = 0x30
    TICKFREQ = 0x31


class SemiHosting:
    """Services semihosting calls made by the guest."""

    def __init__(
        self,
        output: Optional[TextIO] = None,
        input: Optional[TextIO] = None,
        argv: Optional[List[str]] = None,
    ) -> None:
        self._output = output if output is not None else sys.stdout
        self._input = input if input is not None else sys.stdin
        self._argv = list(argv or [])
        self._start = time.monotonic()
        self.exited = False
        """Set once the guest has asked to terminate."""
        self.exit_code = 0
        """Exit status reported by ``SYS_EXIT``."""
        self.unsupported: List[int] = []
        """Operation numbers that were requested but are not implemented."""

    # -- trap handling -------------------------------------------------

    def handle(self, box: Any, mem: Any, reg: Any) -> None:
        """Service one call. ``box`` may be the emulator or ``None`` for pure reads."""
        operation = reg.read("r0")
        parameter = reg.read("r1")
        handler = self._HANDLERS.get(operation)
        if handler is None:
            if operation not in self.unsupported:
                self.unsupported.append(operation)
            logger.warning(f"Semihosting: unsupported operation 0x{operation:02x}")
            result = -1
        else:
            result = handler(self, box, parameter)
        reg.write("r0", result & 0xFFFFFFFF)

    # -- console -------------------------------------------------------

    def _op_write0(self, box: Any, address: int) -> int:
        self._emit(self._read_c_string(box, address))
        return 0

    def _op_writec(self, box: Any, address: int) -> int:
        self._emit(self._read_memory(box, address, 1).decode("latin-1"))
        return 0

    def _op_write(self, box: Any, address: int) -> int:
        handle, buffer, length = self._read_words(box, address, 3)
        data = self._read_memory(box, buffer, length)
        if handle in (1, 2):  # stdout / stderr
            self._emit(data.decode("latin-1"))
            return 0
        logger.debug(f"Semihosting: write to handle {handle} ignored")
        return length

    def _op_readc(self, _box: Any, _parameter: int) -> int:
        char = self._input.read(1)
        return ord(char) if char else -1

    # -- trivial queries ------------------------------------------------

    def _op_istty(self, _box: Any, handle: int) -> int:
        return 1 if handle in (1, 2) else 0

    def _op_flen(self, _box: Any, _handle: int) -> int:
        return -1

    def _op_seek(self, _box: Any, _parameter: int) -> int:
        return 0

    def _op_iserror(self, _box: Any, _status: int) -> int:
        return 0

    def _op_errno(self, _box: Any, _parameter: int) -> int:
        return 0

    def _op_clock(self, _box: Any, _parameter: int) -> int:
        """Centiseconds since the first semihosting call."""
        return int((time.monotonic() - self._start) * 100)

    def _op_time(self, _box: Any, _parameter: int) -> int:
        return int(time.time())

    def _op_elapsed(self, _box: Any, _parameter: int) -> int:
        return 0

    def _op_tickfreq(self, _box: Any, _parameter: int) -> int:
        return 100

    # -- process ---------------------------------------------------------

    def _op_get_cmdline(self, box: Any, address: int) -> int:
        buffer, size = self._read_words(box, address, 2)
        data = " ".join(self._argv).encode("utf-8")
        if size:
            data = data[: max(0, size - 1)]
        data += b"\0"
        self._write_memory(box, buffer, data)
        return len(data) - 1

    def _op_heapinfo(self, box: Any, address: int) -> int:
        # Four words: heap base, heap limit, stack base, stack limit.  The
        # simulator does not track a heap, so report zeros and let the C runtime
        # fall back to its own defaults.
        self._write_memory(box, address, b"\x00" * 16)
        return 0

    def _op_exit(self, box: Any, parameter: int) -> int:
        self.exited = True
        if parameter != ADP_STOPPED_APPLICATION_EXIT and parameter < 0x100:
            self.exit_code = parameter
        logger.info(f"Semihosting: guest exited with code {self.exit_code}")
        if box is not None:
            box.emu_stop()
        return 0

    def _op_exit_extended(self, box: Any, address: int) -> int:
        reason, subcode = self._read_words(box, address, 2)
        if reason == ADP_STOPPED_APPLICATION_EXIT:
            self.exit_code = subcode
        return self._op_exit(box, ADP_STOPPED_APPLICATION_EXIT)

    # -- helpers ---------------------------------------------------------

    def _emit(self, text: str) -> None:
        self._output.write(text)
        self._output.flush()

    @staticmethod
    def _read_memory(box: Any, address: int, size: int) -> bytes:
        try:
            return bytes(box.mem_read(address, size))
        except Exception as err:  # noqa: BLE001 - an unmapped argument is a guest bug
            logger.warning(f"Semihosting: cannot read {size} byte(s) at 0x{address:08x}: {err}")
            return b""

    def _read_words(self, box: Any, address: int, count: int) -> List[int]:
        raw = self._read_memory(box, address, 4 * count)
        return [int.from_bytes(raw[index * 4 : index * 4 + 4], "little") for index in range(len(raw) // 4)]

    @staticmethod
    def _write_memory(box: Any, address: int, data: bytes) -> None:
        try:
            box.mem_write(address, data)
        except Exception as err:  # noqa: BLE001
            logger.warning(f"Semihosting: cannot write {len(data)} byte(s) at 0x{address:08x}: {err}")

    def _read_c_string(self, box: Any, address: int, limit: int = 4096) -> str:
        out = bytearray()
        while len(out) < limit:
            chunk = self._read_memory(box, address + len(out), 1)
            if not chunk or chunk == b"\x00":
                break
            out += chunk
        return out.decode("latin-1")


SemiHosting._HANDLERS: Dict[int, Any] = {
    SemiHostingOp.WRITEC: SemiHosting._op_writec,
    SemiHostingOp.WRITE0: SemiHosting._op_write0,
    SemiHostingOp.WRITE: SemiHosting._op_write,
    SemiHostingOp.READC: SemiHosting._op_readc,
    SemiHostingOp.ISTTY: SemiHosting._op_istty,
    SemiHostingOp.FLEN: SemiHosting._op_flen,
    SemiHostingOp.SEEK: SemiHosting._op_seek,
    SemiHostingOp.ISERROR: SemiHosting._op_iserror,
    SemiHostingOp.ERRNO: SemiHosting._op_errno,
    SemiHostingOp.CLOCK: SemiHosting._op_clock,
    SemiHostingOp.TIME: SemiHosting._op_time,
    SemiHostingOp.ELAPSED: SemiHosting._op_elapsed,
    SemiHostingOp.TICKFREQ: SemiHosting._op_tickfreq,
    SemiHostingOp.GET_CMDLINE: SemiHosting._op_get_cmdline,
    SemiHostingOp.HEAPINFO: SemiHosting._op_heapinfo,
    SemiHostingOp.EXIT: SemiHosting._op_exit,
    SemiHostingOp.EXIT_EXTENDED: SemiHosting._op_exit_extended,
}
