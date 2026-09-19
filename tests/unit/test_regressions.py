# -*- coding: utf-8 -*-

"""Regression tests for defects found during the code analysis.

Every test here maps to a concrete bug that used to reproduce; they are the
"red before / green after" record for the P0 fixes.
"""

import pytest

from xuanwu.exception import XwInvalidMemoryAddress, XwInvalidMemorySize, XwInvalidParameter


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


class TestErrorMessagesInterpolate:
    """Several raise sites were missing the f prefix, printing literal {name}."""

    def test_missing_chip_file_reports_the_path(self, stm32f411_firmware):
        from xuanwu import XuanWu

        bogus = "/nonexistent/dir/chip.yaml"
        with pytest.raises(XwInvalidParameter) as excinfo:
            XuanWu(bogus, str(stm32f411_firmware))
        assert bogus in str(excinfo.value)
        assert "{chip}" not in str(excinfo.value)
