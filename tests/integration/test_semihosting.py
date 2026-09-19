# -*- coding: utf-8 -*-

"""The semihosting trap has to work through the real emulator.

The guest reaches semihosting by executing ``BKPT 0xAB``.  Unicorn reports that
as an ordinary ``BKPT`` and leaves the PC on the instruction, so the controller
recognises the trap, services it and steps over it.  These tests run genuine
code through Unicorn to prove all three parts happen.
"""

import io

import pytest

from xuanwu import XuanWu
from xuanwu.backends import SEMIHOST_BKPT, SemiHosting

pytestmark = pytest.mark.integration

CODE = 0x2001_F000
STRING = 0x2001_F100

# movs r0, #4      SYS_WRITE0
# adr  r1, #4      -> STRING (PC at +4, so (0x100 - 4) / 4 = 63)
# bkpt 0xab        the trap
# b    .           park here
PROGRAM = bytes.fromhex(
    "0420"    # movs r0, #4
    "3fa1"    # adr  r1, #252      (0x100 - 4) / 4 = 63 -> 0xA13F
    "abbe"    # bkpt 0xab
    "fee7"    # b .
)
PLAIN_BKPT = bytes.fromhex("0020" "00be" "fee7")  # movs r0, #0 ; bkpt 0x00 ; b .


@pytest.fixture
def device(stm32f411_path, stm32f411_firmware):
    """A fresh device per test: the snippet below hijacks the program counter."""
    stream = io.StringIO()
    host = SemiHosting(output=stream)
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware), semihosting=host)
    device.reset()
    return device, host, stream


def install(device, program: bytes) -> None:
    device.mem.write(CODE, program)
    device.mem.write(STRING, b"hello from semihosting\n\x00")
    device.reg.write("pc", CODE | 0x1)  # Thumb


def test_bkpt_0xab_is_serviced_and_stepped_over(device):
    device, host, stream = device
    install(device, PROGRAM)

    device.run(count=3)  # movs, adr, bkpt

    assert stream.getvalue() == "hello from semihosting\n"
    # The trap must be stepped over: the PC is now on `b .`, not on the BKPT.
    assert device.reg.pc == CODE + 4 + 2
    assert host.exited is False


def test_the_guest_keeps_running_after_the_trap(device):
    device, _host, stream = device
    install(device, PROGRAM)

    device.run(count=4)  # one more instruction: the `b .` loop

    assert stream.getvalue() == "hello from semihosting\n"
    assert device.reg.pc == CODE + 6


def test_the_trap_does_not_leave_thumb_state(device):
    """Stepping the PC over the ``BKPT`` must keep bit 0 set.

    Writing an even PC switches Unicorn to Arm state, so the next fetch -- the
    ``b .`` here -- fails with ``UC_ERR_INSN_INVALID``.
    """
    from unicorn import arm_const as uc_arm

    device, _host, _stream = device
    install(device, PROGRAM)

    device.run(count=3)
    assert device.reg.read(uc_arm.UC_ARM_REG_CPSR) & 0x20, "the core left Thumb state"

    device.run(count=1)  # the `b .` loop: used to raise UC_ERR_INSN_INVALID
    assert device.reg.pc == CODE + 6


def test_a_plain_bkpt_is_still_a_fault(device):
    """Only the 0xAB immediate is a semihosting call."""
    device, _host, stream = device
    install(device, PLAIN_BKPT)

    with pytest.raises(BaseException):
        device.run(count=2)
    assert stream.getvalue() == ""


def test_semihosting_can_be_disabled(stm32f411_path, stm32f411_firmware):
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware), semihosting=False)
    device.reset()
    install(device, PROGRAM)

    with pytest.raises(BaseException):
        device.run(count=3)


def test_the_trap_immediate_matches_the_encoding():
    # Guards the constant the controller compares against.
    assert SEMIHOST_BKPT == 0xBEAB
    assert int.from_bytes(bytes.fromhex("abbe"), "little") == SEMIHOST_BKPT
