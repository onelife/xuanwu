# -*- coding: utf-8 -*-

"""Unit tests for the floating-point extension."""

import pytest

from xuanwu.arch.base import arm_core_registers
from xuanwu.arch.cortex_m import FP_FRAME_SIZE, CORE_PERIPHERALS


class TestRegisterFile:
    def test_fp_registers_are_addressable_by_name(self):
        for index in range(32):
            assert f"s{index}" in arm_core_registers
        for index in range(16):
            assert f"d{index}" in arm_core_registers

    def test_single_registers_are_readable_and_writable(self, stm32f411_with_firmware):
        reg = stm32f411_with_firmware.reg
        reg.write("s0", 0x3FC00000)  # 1.5f
        assert reg.read("s0") == 0x3FC00000
        reg.write("s1", 0x40400000)  # 3.0f
        # D0 aliases the S1:S0 pair
        assert reg.read("d0") == 0x404000003FC00000

    def test_fpscr_is_readable_and_writable(self, stm32f411_with_firmware):
        reg = stm32f411_with_firmware.reg
        reg.write("fpscr", 0x00000010)
        assert reg.read("fpscr") == 0x00000010


class TestExtensionRegisters:
    @pytest.fixture
    def fpu(self, stm32f411_with_firmware):
        return stm32f411_with_firmware.hw.perif["fpu"]

    def test_the_model_is_registered(self):
        assert CORE_PERIPHERALS["fpu"] is not None

    def test_feature_registers_report_single_precision(self, fpu):
        assert fpu.read_register("MVFR0") == 0x10110021
        assert fpu.read_register("MVFR1") == 0x11000011
        assert fpu.read_register("MVFR2") == 0x00000000

    def test_feature_registers_are_read_only(self, fpu):
        fpu.write(0xE000EF40, 4, 0xFFFFFFFF)
        assert fpu.read_register("MVFR0") == 0x10110021

    def test_fpccr_resets_with_aspen_and_lspen(self, fpu):
        assert fpu.read_register("FPCCR") == 0xC0000000

    def test_lspact_cannot_be_written_by_the_guest(self, fpu):
        # Lazy stacking is not modelled, so the state bit must never be settable
        # through the register interface.
        fpu.write(0xE000EF34, 4, 0xC0000001)
        assert fpu.read_register("FPCCR") & 0x1 == 0
        assert fpu.read(0xE000EF34, 4) & 0x1 == 0

    def test_fpccr_bits_are_writable(self, fpu):
        fpu.write(0xE000EF34, 4, 0xC0000002)  # USER
        assert fpu.read(0xE000EF34, 4) & (1 << 1)

    def test_fpcar_is_aligned_and_masked(self, fpu):
        fpu.write(0xE000EF38, 4, 0xFFFFFFFF)
        assert fpu.read_register("FPCAR") == 0xFFFFFFF8

    def test_reset_restores_the_defaults(self, fpu):
        fpu.write_register("FPCCR", 0)
        fpu.write_register("FPDSCR", 0x07000000)
        fpu.reset()
        assert fpu.read_register("FPCCR") == 0xC0000000
        assert fpu.read_register("FPDSCR") == 0


class TestExceptionFrame:
    def test_fp_frame_is_bigger_than_the_basic_one(self):
        # 8-word basic frame + S0-S15 + FPSCR + reserved
        assert FP_FRAME_SIZE == 0x20 + 18 * 4

    def test_the_reserved_window_is_not_claimed_by_another_model(self, stm32f411_with_firmware):
        device = stm32f411_with_firmware
        for address in (0xE000EF34, 0xE000EF38, 0xE000EF3C, 0xE000EF40, 0xE000EF44):
            owners = [record.desc for record in device.mem.get_io(address, address + 4)]
            assert owners == ["FPU"], f"0x{address:08x} is claimed by {owners}"

    def test_the_chip_without_an_fpu_does_not_declare_one(self, sam3x8e_path, sam3x8e_firmware):
        from xuanwu import XuanWu

        device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware))
        assert "fpu" not in device.hw.perif
