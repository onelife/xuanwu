# -*- coding: utf-8 -*-

"""The unclaimed-MMIO report.

Booting an unfamiliar firmware leaves a trail of accesses that no peripheral
model claimed. That trail is the work list for bringing a chip up, so it is
collected and can be printed instead of only scrolling past as warnings.
"""

import logging

import pytest

from xuanwu import XuanWu

pytestmark = pytest.mark.integration

INSTRUCTIONS = 200_000


@pytest.fixture(scope="module")
def booted(stm32f411_path, stm32f411_firmware):
    logging.disable(logging.CRITICAL)
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    device.reset()
    device.run(count=INSTRUCTIONS)
    return device


def test_report_is_empty_before_running(stm32f411_path, stm32f411_firmware):
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    assert device.mem.unclaimed_accesses() == []


def test_booting_records_unclaimed_peripherals(booted):
    records = booted.mem.unclaimed_accesses()
    assert records, "the STM32F411 firmware touches peripherals that are not modelled"

    for address, size, count in records:
        assert size in (1, 2, 4)
        assert count >= 1
    addresses = [address for address, _size, _count in records]
    assert addresses == sorted(addresses)


def test_the_flash_interface_shows_up(booted):
    """0x40023C00 is the STM32F4 FLASH register block, which is not modelled."""
    addresses = {address for address, _size, _count in booted.mem.unclaimed_accesses()}
    assert 0x40023C00 in addresses


def test_repeated_accesses_are_counted(booted):
    counts = {address: count for address, _size, count in booted.mem.unclaimed_accesses()}
    assert counts.get(0x40023C00, 0) > 1


def test_show_unclaimed_prints_a_table(booted, capsys):
    booted.mem.show_unclaimed()
    out = capsys.readouterr().out
    assert "Address" in out and "Count" in out
    assert "0x40023c00" in out
    assert "unclaimed address(es)" in out


def test_show_unclaimed_handles_the_empty_case(stm32f411_path, stm32f411_firmware, capsys):
    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    device.mem.show_unclaimed()
    assert "every MMIO access so far was claimed" in capsys.readouterr().out
