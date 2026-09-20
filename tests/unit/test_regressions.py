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
        # NVIC_IPR0 holds four priority bytes; with two implemented bits each -- the
        # *top* two, as the architecture has it -- writing all ones reads back as 0xC0
        # per byte.  This used to assert 0xFC/byte, which is what the mask produced
        # when it kept the low bits instead of the implemented ones.
        nvic.write(0xE000E400, 4, 0xFFFF_FFFF)
        assert nvic.read(0xE000E400, 4) == 0xC0C0_C0C0
        # And the write has to land on IPR0, which is the register NVIC_SetPriority()
        # writes: it used to land on IPR32, because the table put IPR0 at 0x280.
        assert nvic.read_register("IPR0") == 0xC0C0_C0C0
        assert nvic.get_priority(0) == 0xC0
        assert nvic.get_priority(128) == 0


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


# ---------------------------------------------------------------------------
# The register-table and engine defects found while porting the simulator to Go
# (docs/plan-go.md, appendix D.1).  They are here rather than in a new file
# because each one is the same kind of record: a test written for a bug that used
# to reproduce.
# ---------------------------------------------------------------------------


class TestNvicRegisterLayout:
    """The NVIC template left one reserved block too few, so IPR0 sat at 0x280.

    A driver's ``NVIC_SetPriority()`` writes ``0xE000E400`` -- the architectural
    IPR0 -- which the model read as IPR32, so the priority the guest set was not the
    priority the model scheduled on.
    """

    def test_the_register_table_uses_the_architectural_offsets(self, box):
        nvic = box.hw.perif["nvic"]
        # Offsets from ISER0 at 0xE000E100 (Armv7-M 4.2.2).  IPR0 is a whole block
        # after IABR0, which is what the template used to get wrong.
        expected = {
            "ISER0": 0x00,
            "ISER7": 0x1C,
            "ICER0": 0x80,
            "ISPR0": 0x100,
            "ICPR0": 0x180,
            "IABR0": 0x200,
            "IPR0": 0x300,
            "IPR59": 0x3EC,
        }
        for name, offset in expected.items():
            _format, offset_, _mask = nvic.registers[name]
            assert offset_ == offset, f"{name} is at 0x{offset_:X}, want 0x{offset:X}"

        # The whole table is the window the description declares: 0x300 plus the 60
        # priority words of a 240-interrupt part.
        size = sum(register.format.size for register in nvic.registers.values())
        assert size == 0x3F0

    def test_a_priority_write_reaches_the_priority_the_model_uses(self, box):
        nvic = box.hw.perif["nvic"]
        nvic.write(0xE000_E400, 4, 0x0000_00C0)  # priority of IRQ 0
        assert nvic.get_priority(0) == 0xC0
        assert nvic.read_register("IPR0") == 0x0000_00C0

    def test_where_ipr0_used_to_be_is_reserved_now(self, box):
        nvic = box.hw.perif["nvic"]
        assert nvic.read(0xE000_E380, 4) == 0


class TestNvicRegisterIndexParsing:
    """``int(name[-1])`` read ISER10 as ISER0."""

    def test_a_two_digit_block_index_is_parsed(self):
        from xuanwu.arch.cortex_m.nvic import ArmHardwareNvic

        assert ArmHardwareNvic._block_index("ISER0") == 0
        assert ArmHardwareNvic._block_index("ISER10") == 10
        assert ArmHardwareNvic._block_index("ICPR15") == 15
        assert ArmHardwareNvic._block_index("IPR3") == 3


class TestNvicClearPending:
    """ICPR's write mask was zero, so NVIC_ClearPendingIRQ() was a no-op."""

    def test_writing_icpr_clears_the_bit_and_the_engine(self, box):
        box.reset()
        hw = box.hw
        nvic = hw.perif["nvic"]
        hw.set_irq_pending(2)
        nvic.write_register("ISPR0", 0x0000_0004)
        assert hw.is_irq_pending_or_active(2)

        nvic.write(0xE000_E280, 4, 0x0000_0004)  # NVIC_ClearPendingIRQ(2)

        assert not nvic.read_register("ISPR0") & 0x0000_0004
        assert not hw.is_irq_pending_or_active(2)


class TestReservedRegions:
    """A reserved gap was one multi-word ``Struct``: writing it raised."""

    def test_a_reserved_word_reads_zero_and_drops_writes(self, box):
        nvic = box.hw.perif["nvic"]
        # RESERVED0 is the gap between ISER7 and ICER0: 24 words at 0xE000E120.
        assert nvic.read(0xE000_E120, 4) == 0  # the first word of the gap
        assert nvic.read(0xE000_E148, 4) == 0  # and one in the middle of it
        nvic.write(0xE000_E120, 4, 0xFFFF_FFFF)  # used to raise struct.error
        nvic.write(0xE000_E148, 4, 0xFFFF_FFFF)  # used to raise XwInvalidMemoryAddress
        assert nvic.read(0xE000_E120, 4) == 0
        assert nvic.read(0xE000_E148, 4) == 0

    def test_an_access_that_crosses_a_word_is_still_a_size_error(self, box):
        nvic = box.hw.perif["nvic"]
        with pytest.raises(XwInvalidMemorySize):
            nvic.read(0xE000_E120, 8)


class TestVectorTableBase:
    """``jump_isr`` OR-ed the vector base with the exception offset.

    That is the same thing as adding only while bits 7..9 of the base are clear; a
    table at 0x20000200 fetched the wrong vector for every exception from IRQ 112 up.
    """

    def test_a_base_that_shares_bits_with_the_offset(self, box):
        box.reset()
        scb = box.hw.perif["scb"]
        base = 0x2000_0200  # 32-byte aligned, and bit 9 is already set
        scb.write_register("VTOR", base)
        # IRQ 112 is exception 128, so its vector is the word at base + 128 * 4.
        box.mem.write(base + 128 * 4, (0x0000_1235).to_bytes(4, "little"))
        box.hw.jump_isr(112)
        assert box.reg.pc == 0x0000_1234
        assert box.reg.read("ipsr") == 128


class TestTailChainingIRQ0:
    """``if next_irq:`` skipped tail-chaining when the next exception was IRQ 0."""

    def test_irq_zero_is_tail_chained(self, box):
        from xuanwu.config import EXCP

        box.reset()
        hw = box.hw
        nvic = hw.perif["nvic"]
        # The IRQ 0 handler, in the vector table at address 0.
        box.mem.write((16 + 0) * 4, (0x0000_2101).to_bytes(4, "little"))
        nvic.write_register("ISPR0", 0)
        nvic.write_register("IABR0", 0)
        hw._irq_pending.clear()
        hw._irq_handling.clear()
        nvic.write_register("ISER0", 1)
        hw.set_irq_pending(0)
        assert hw.dispatch_pending_exception() is True
        assert hw._reg.pc == 0x0000_2100

        # Pending again while its handler runs: the return from the handler has to
        # tail-chain into it instead of unstacking back to thread mode.
        hw.set_irq_pending(0)
        hw._reg.write("lr", 0xFFFF_FFF9)
        hw._reg.pc_t = 0xFFFF_FFF9
        hw.system_interrupt_callback(box.box, EXCP.EXCEPTION_EXIT, None)

        assert 0 in hw._irq_handling
        assert hw._reg.pc == 0x0000_2100
        assert hw._reg.read("ipsr") == 16


class TestResetForgetsWhatTheEngineWasDoing:
    """``reset()`` reset the models but left the engine's lists alone."""

    def test_reset_clears_pending_and_handling(self, box):
        box.reset()
        hw = box.hw
        nvic = hw.perif["nvic"]
        hw.set_irq_pending(3)
        hw._irq_handling.append(5)  # as if a handler were running
        hw._thread_mode = False
        assert nvic.read_register("ISPR0") & (1 << 3)

        hw.reset()

        assert hw._irq_pending == []
        assert hw._irq_handling == []
        assert hw._thread_mode is True
        # The models went back to their reset values as well, which is what the
        # engine's lists would otherwise still be pointing at.
        assert nvic.read_register("ISPR0") == 0
        assert nvic.read_register("IABR0") == 0


class TestUnhandledCoreExceptions:
    """The interrupt callback ended with a bare ``raise``.

    Python turns that into "RuntimeError: No active exception to reraise", which says
    nothing about what the guest did -- and it is what an ``svc`` instruction, or a
    ``bkpt`` with semihosting switched off, used to produce.
    """

    def test_svc_pends_svcall(self, box):
        from xuanwu.arch.cortex_m.constants import Exception_
        from xuanwu.config import EXCP

        hw = box.hw
        hw.clear_irq_pending(Exception_.SVCall - 16)
        box.hw.system_interrupt_callback(box.box, EXCP.SWI, None)
        assert hw.is_irq_pending_or_active(Exception_.SVCall - 16)

    def test_a_plain_bkpt_is_reported_as_unsupported(self, box):
        from xuanwu.config import EXCP
        from xuanwu.exception import XwUnsupported

        with pytest.raises(XwUnsupported):
            box.hw.system_interrupt_callback(box.box, EXCP.BKPT, None)

    def test_an_unknown_core_exception_is_reported(self, box):
        from xuanwu.exception import XwUnsupported

        with pytest.raises(XwUnsupported):
            box.hw.system_interrupt_callback(box.box, 99, None)
