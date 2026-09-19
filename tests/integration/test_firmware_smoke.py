# -*- coding: utf-8 -*-

"""Smoke tests: every bundled test firmware must load and execute.

The firmware parametrisation comes from ``pytest_generate_tests`` in conftest.
"""

import shutil

import pytest

from xuanwu import XuanWu

pytestmark = pytest.mark.integration

INSTRUCTIONS = 200_000
HAS_SOCAT = shutil.which("socat") is not None


def test_firmware_loads_and_runs(firmware, chip_paths):
    path, chip = firmware
    if chip == "sam3x8e" and not HAS_SOCAT:
        pytest.skip("sam3x8e needs the socat serial bridge")

    device = XuanWu(str(chip_paths[chip]), str(path))
    device.reset()

    # Reset must have taken the vector table into account.
    assert device.reg.msp != 0
    assert device.reg.pc != 0

    device.run(count=INSTRUCTIONS)
    # Execution must stay inside a 32-bit address space.
    assert 0 < device.reg.pc < 0x1_0000_0000


def test_chip_can_be_selected_by_name(stm32f411_firmware):
    """The chip argument accepts a bundled name as well as a path."""
    device = XuanWu("stm32f411", str(stm32f411_firmware))
    assert device._chip["name"] == "stm32f411"


def test_unknown_firmware_extension_is_rejected(stm32f411_path, tmp_path):
    from xuanwu.exception import XwInvalidParameter

    bogus = tmp_path / "firmware.xyz"
    bogus.write_bytes(b"\x00")
    with pytest.raises(XwInvalidParameter) as excinfo:
        XuanWu(str(stm32f411_path), str(bogus))
    assert "xyz" in str(excinfo.value)
