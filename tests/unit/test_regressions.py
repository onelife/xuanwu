# -*- coding: utf-8 -*-

"""Regression tests for defects found during the code analysis.

Every test here maps to a concrete bug that used to reproduce; they are the
"red before / green after" record for the P0 fixes.
"""

import pytest

from xuanwu.exception import XwInvalidMemoryAddress, XwInvalidMemorySize, XwInvalidParameter

from xuanwu.arch.cortex_m.constants import Exception_, SHCSR


@pytest.fixture(scope="module")
def box(stm32f411_with_firmware):
    return stm32f411_with_firmware


class TestPeripheralAccessOutOfRange:
    """ArmHardwareBase.read/write used to check an uninitialised `data_orig`."""

    def test_read_past_end_of_block_raises_domain_error(self, box):
        with pytest.raises(XwInvalidMemoryAddress):
            box.hw.perif["scid"].read(0xE000E010, 4)

    def test_write_past_end_of_block_raises_domain_error(self, box):
        with pytest.raises(XwInvalidMemoryAddress):
            box.hw.perif["scid"].write(0xE000E010, 4, 1)

    def test_oversized_access_raises_size_error(self, box):
        with pytest.raises(XwInvalidMemorySize):
            box.hw.perif["scid"].read(0xE000E000, 8)


class TestNvicWriteOnlyRegisters:
    """fix_after_read/fix_before_write assigned to an index of a str."""

    def test_read_icer0_returns_iser0(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write_register("ISER0", 0x0000_0010)
        assert nvic.read(0xE000E180, 4) == 0x0000_0010

    def test_read_icpr0_returns_ispr0(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write_register("ISPR0", 0x0000_0004)
        assert nvic.read(0xE000E280, 4) == 0x0000_0004

    def test_write_icer0_clears_iser0(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write_register("ISER0", 0xFFFF_FFFF)
        nvic.write(0xE000E180, 4, 0x0000_0003)
        assert nvic.read_register("ISER0") == 0xFFFF_FFFC


class TestNvicPendingAndActiveBits:
    """set_pending/set_active flipped the bit in the wrong direction."""

    def test_set_pending_sets_the_bit(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write_register("ISPR0", 0)
        nvic.set_pending(9, state=True)
        assert nvic.read_register("ISPR0") & (1 << 9)

    def test_clear_pending_clears_the_bit(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.set_pending(9, state=True)
        nvic.set_pending(9, state=False)
        assert not nvic.read_register("ISPR0") & (1 << 9)

    def test_set_active_sets_the_bit(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write_register("IABR0", 0)
        nvic.set_active(9, state=True)
        assert nvic.read_register("IABR0") & (1 << 9)

    def test_clear_active_clears_the_bit(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.set_active(9, state=True)
        nvic.set_active(9, state=False)
        assert not nvic.read_register("IABR0") & (1 << 9)


class TestScbResetValues:
    def test_scr_reset_value_has_no_vectkey_leftovers(self, box):
        # SCR only defines bits 1, 2, 4 and 8; the old reset value was AIRCR's 0xFA050000.
        assert box.hw.perif["scb"].read_register("SCR") == 0

    def test_cpuid_comes_from_the_chip_description(self, box):
        # stm32f411.yaml declares cpuid: 0x410FC240
        assert box.hw.perif["scb"].read_register("CPUID") == 0x410FC240


class TestProgramCounterWritesKeepThumbState:
    """Writing an even PC dropped Unicorn into Arm state.

    The next fetch then failed with ``UC_ERR_INSN_INVALID``.  It was found
    through the semihosting trap, which steps the PC over the ``BKPT`` by hand.
    """

    CODE = 0x2001_F000

    @pytest.fixture
    def device(self, stm32f411_path, stm32f411_firmware):
        from xuanwu import XuanWu

        device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
        device.reset()
        return device

    def test_pc_t_keeps_the_instruction_set_bit(self, device):
        from unicorn import arm_const as uc_arm

        device.reg.pc_t = 0x2000_0000
        assert device.reg.pc == 0x2000_0000
        assert device.reg.read(uc_arm.UC_ARM_REG_CPSR) & 0x20, "the core left Thumb state"

    def test_the_core_keeps_fetching_after_a_pc_t_write(self, device):
        device.mem.write(self.CODE, bytes.fromhex("00bf" "00bf" "00bf" "fee7"))  # nop x3, b .
        device.reg.pc_t = self.CODE

        device.run(count=4)  # must not raise UC_ERR_INSN_INVALID
        assert device.reg.pc == self.CODE + 6


class TestRunHonoursTheInstructionBudget:
    """``run(count=N)`` used to pass N to Unicorn as a *timeout* in milliseconds."""

    CODE = 0x2001_F000

    @pytest.fixture
    def device(self, stm32f411_path, stm32f411_firmware):
        from xuanwu import XuanWu

        device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
        device.reset()
        return device

    def test_count_stops_after_exactly_that_many_instructions(self, device):
        device.mem.write(self.CODE, bytes.fromhex("00bf" "00bf" "00bf" "fee7"))  # nop x3, b .
        device.reg.pc_t = self.CODE

        device.run(count=1)
        assert device.reg.pc == self.CODE + 2
        device.run(count=2)
        assert device.reg.pc == self.CODE + 6


class TestNvicRegisterTableIsPerInstance:
    """``ArmHardwareNvic.REGISTERS`` was a class attribute rewritten in ``__init__``.

    Each instance did build its own table, but the class attribute was left
    pointing at whichever table was built last, so an already-created controller
    reported the wrong register list as soon as a second one existed.
    """

    def make(self, chip_path, firmware, **options):
        from xuanwu import XuanWu

        return XuanWu(str(chip_path), str(firmware), hardware_options=options or None)

    def test_two_controllers_keep_their_own_table(self, stm32f411_path, stm32f411_firmware):
        from xuanwu.arch.cortex_m import ArmHardwareNvic

        wide = self.make(stm32f411_path, stm32f411_firmware)
        narrow = self.make(stm32f411_path, stm32f411_firmware, interrupt_lines=1, priority_bits=2)

        wide_nvic = wide.hw.perif["nvic"]
        narrow_nvic = narrow.hw.perif["nvic"]
        assert len(wide_nvic.registers) != len(narrow_nvic.registers)
        # REGISTERS must describe the same table as the built register file, for
        # both instances, at the same time.
        assert len(wide_nvic.REGISTERS) == len(wide_nvic.registers)
        assert len(narrow_nvic.REGISTERS) == len(narrow_nvic.registers)
        assert ArmHardwareNvic.REGISTERS == (), "no table may be cached on the class"

    def test_the_priority_mask_follows_priority_bits(self, stm32f411_path, stm32f411_firmware):
        device = self.make(stm32f411_path, stm32f411_firmware, priority_bits=2)
        nvic = device.hw.perif["nvic"]
        # NVIC_IPR0 holds four priority bytes; with two implemented bits each,
        # writing all ones must read back as 0xFC per byte.
        nvic.write(0xE000E400, 4, 0xFFFF_FFFF)
        assert nvic.read(0xE000E400, 4) == 0xFCFC_FCFC


class TestScbActiveBits:
    """``ArmHardwareScb.set_active()`` flipped the bit in the wrong direction.

    The NVIC had the same defect (see below); the SCB version survived until the
    Arduino ``micros()`` formula, which reads ``SHCSR.SYSTICKACT`` as its
    "a tick is due" term, was checked against the registers the guest reads.
    """

    def test_setting_active_sets_the_bit(self, box):
        scb = box.hw.perif["scb"]
        scb.write_register("SHCSR", 0)
        scb.set_active(Exception_.SysTick, state=True)
        assert scb.read_register("SHCSR") & (1 << SHCSR.SYSTICKACT)

    def test_clearing_active_clears_the_bit(self, box):
        scb = box.hw.perif["scb"]
        scb.set_active(Exception_.SysTick, state=True)
        scb.set_active(Exception_.SysTick, state=False)
        assert not scb.read_register("SHCSR") & (1 << SHCSR.SYSTICKACT)

    def test_each_exception_has_its_own_bit(self, box):
        scb = box.hw.perif["scb"]
        scb.write_register("SHCSR", 0)
        scb.set_active(Exception_.PendSV, state=True)
        assert scb.read_register("SHCSR") & (1 << SHCSR.PENDSVACT)
        assert not scb.read_register("SHCSR") & (1 << SHCSR.SYSTICKACT)


class TestErrorMessagesInterpolate:
    """Several raise sites were missing the f prefix, printing literal {name}."""

    def test_missing_chip_file_reports_the_path(self, stm32f411_firmware):
        from xuanwu import XuanWu

        bogus = "/nonexistent/dir/chip.yaml"
        with pytest.raises(XwInvalidParameter) as excinfo:
            XuanWu(bogus, str(stm32f411_firmware))
        assert bogus in str(excinfo.value)
        assert "{chip}" not in str(excinfo.value)
