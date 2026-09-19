# -*- coding: utf-8 -*-

"""Unit tests for the semihosting services."""

import io

import pytest

from xuanwu.backends import SemiHosting, SemiHostingOp

BASE = 0x1000


class FakeBox:
    """Just enough of a Unicorn engine for the semihosting services."""

    def __init__(self) -> None:
        self.memory = bytearray(0x20000)
        self.stopped = False

    def mem_read(self, address: int, size: int) -> bytes:
        if address + size > len(self.memory):
            raise ValueError(f"unmapped read at 0x{address:x}")
        return bytes(self.memory[address : address + size])

    def mem_write(self, address: int, data: bytes) -> None:
        self.memory[address : address + len(data)] = data

    def emu_stop(self) -> None:
        self.stopped = True

    def put(self, address: int, data: bytes) -> None:
        self.mem_write(address, data)

    def string_at(self, address: int) -> str:
        end = self.memory.index(0, address)
        return self.memory[address:end].decode("latin-1")

    def words_at(self, address: int, count: int) -> list:
        return [int.from_bytes(self.memory[address + 4 * i : address + 4 * i + 4], "little") for i in range(count)]


class FakeReg:
    def __init__(self) -> None:
        self.registers = {"r0": 0, "r1": 0}

    def read(self, name: str) -> int:
        return self.registers[name]

    def write(self, name: str, value: int) -> None:
        self.registers[name] = value


@pytest.fixture
def services():
    box, reg = FakeBox(), FakeReg()
    stream = io.StringIO()
    host = SemiHosting(output=stream, input=io.StringIO("Q"), argv=["firmware.elf", "--flag"])
    return host, box, reg, stream


def call(host, box, reg, operation, parameter=0):
    reg.write("r0", int(operation))
    reg.write("r1", parameter)
    host.handle(box, None, reg)
    return reg.read("r0")


class TestConsole:
    def test_write0_prints_a_c_string(self, services):
        host, box, reg, stream = services
        box.put(BASE, b"hello\0trailing")
        assert call(host, box, reg, SemiHostingOp.WRITE0, BASE) == 0
        assert stream.getvalue() == "hello"

    def test_writec_prints_one_character(self, services):
        host, box, reg, stream = services
        box.put(BASE, b"A")
        assert call(host, box, reg, SemiHostingOp.WRITEC, BASE) == 0
        assert stream.getvalue() == "A"

    def test_write_prints_a_buffer(self, services):
        host, box, reg, stream = services
        box.put(BASE, b"payload")
        box.put(BASE + 16, (1).to_bytes(4, "little") + (BASE).to_bytes(4, "little") + (7).to_bytes(4, "little"))
        assert call(host, box, reg, SemiHostingOp.WRITE, BASE + 16) == 0
        assert stream.getvalue() == "payload"

    def test_write_to_a_non_console_handle_reports_the_unsent_bytes(self, services):
        host, box, reg, stream = services
        box.put(BASE, b"data")
        box.put(BASE + 16, (3).to_bytes(4, "little") + (BASE).to_bytes(4, "little") + (4).to_bytes(4, "little"))
        assert call(host, box, reg, SemiHostingOp.WRITE, BASE + 16) == 4
        assert stream.getvalue() == ""

    def test_readc_takes_a_byte_from_the_input(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.READC) == ord("Q")
        assert call(host, box, reg, SemiHostingOp.READC) == 0xFFFFFFFF  # -1 at end of input


class TestQueries:
    @pytest.mark.parametrize("handle,expected", [(1, 1), (2, 1), (3, 0)])
    def test_istty(self, services, handle, expected):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.ISTTY, handle) == expected

    def test_clock_counts_centiseconds(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.CLOCK) >= 0

    def test_time_is_a_unix_timestamp(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.TIME) > 1_600_000_000

    def test_errno_and_iserror_are_zero(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.ERRNO) == 0
        assert call(host, box, reg, SemiHostingOp.ISERROR, 1) == 0

    def test_unknown_file_operations_report_failure(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.OPEN, BASE) == 0xFFFFFFFF
        assert call(host, box, reg, SemiHostingOp.FLEN, 1) == 0xFFFFFFFF


class TestProcess:
    def test_get_cmdline_writes_the_argv(self, services):
        host, box, reg, _stream = services
        box.put(BASE, (BASE + 32).to_bytes(4, "little") + (64).to_bytes(4, "little"))
        length = call(host, box, reg, SemiHostingOp.GET_CMDLINE, BASE)
        assert box.string_at(BASE + 32) == "firmware.elf --flag"
        assert length == len("firmware.elf --flag")

    def test_heapinfo_writes_four_zero_words(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.HEAPINFO, BASE) == 0
        assert box.words_at(BASE, 4) == [0, 0, 0, 0]

    def test_exit_stops_the_emulator(self, services):
        host, box, reg, _stream = services
        assert box.stopped is False
        call(host, box, reg, SemiHostingOp.EXIT, 0x20026)  # ADP_Stopped_ApplicationExit
        assert host.exited is True
        assert host.exit_code == 0
        assert box.stopped is True

    def test_exit_with_a_small_code_records_it(self, services):
        host, box, reg, _stream = services
        call(host, box, reg, SemiHostingOp.EXIT, 3)
        assert host.exit_code == 3

    def test_exit_extended_reads_the_reason_block(self, services):
        host, box, reg, _stream = services
        box.put(BASE, (0x20026).to_bytes(4, "little") + (7).to_bytes(4, "little"))
        call(host, box, reg, SemiHostingOp.EXIT_EXTENDED, BASE)
        assert host.exited is True
        assert host.exit_code == 7
        assert box.stopped is True


class TestRobustness:
    def test_unsupported_operation_returns_minus_one_and_is_recorded(self, services):
        host, box, reg, _stream = services
        assert call(host, box, reg, SemiHostingOp.RENAME, BASE) == 0xFFFFFFFF
        assert call(host, box, reg, SemiHostingOp.RENAME, BASE) == 0xFFFFFFFF
        assert host.unsupported == [SemiHostingOp.RENAME]

    def test_unmapped_argument_does_not_raise(self, services):
        host, box, reg, stream = services
        assert call(host, box, reg, SemiHostingOp.WRITE0, 0xFFFFF0) == 0
        assert stream.getvalue() == ""
