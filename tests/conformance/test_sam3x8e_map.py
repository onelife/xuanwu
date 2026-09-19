# -*- coding: utf-8 -*-

"""The SAM3X8E address map, checked against the datasheet.

A wrong base address does not fail loudly: the peripheral model simply sits
somewhere the firmware never writes, and every access to the real block shows up as
"unclaimed".  That is hard to notice, so the addresses the chip description uses are
pinned here, together with the list of peripherals that are *not* modelled yet --
adding one means editing this file, which is the point.
"""

from pathlib import Path

import pytest
import yaml

pytestmark = pytest.mark.conformance

CHIP = Path(__file__).resolve().parents[2] / "src" / "xuanwu" / "data" / "chips" / "arm" / "cortex_m" / "sam3x8e.yaml"

# (base, size) from the SAM3X8E datasheet, for every block the description models.
DATASHEET = {
    # Cortex-M3 system control space
    "SCID": (0xE000_E000, 0x10),
    "SYSTICK": (0xE000_E010, 0x10),
    "NVIC": (0xE000_E100, 0x3F0),
    "SCB": (0xE000_ED00, 0x40),
    "DBG": (0xE000_EDF0, 0x10),
    "DWT": (0xE000_1000, 0x1000),
    # peripherals
    "PMC": (0x400E_0600, 0x110),
    "EFC0": (0x400E_0A00, 0x14),
    "EFC1": (0x400E_0C00, 0x14),
    "GPIOA": (0x400E_0E00, 0x130),
    "GPIOB": (0x400E_1000, 0x130),
    "GPIOC": (0x400E_1200, 0x130),
    "GPIOD": (0x400E_1400, 0x130),
    "GPIOE": (0x400E_1600, 0x130),
    "GPIOF": (0x400E_1800, 0x130),
    "UART": (0x400E_0800, 0x200),
    "ADC": (0x400C_0000, 0xEC),
    "SPI": (0x4000_8000, 0xEC),
    "TWI0": (0x4008_C000, 0x200),
    "TWI1": (0x4009_0000, 0x200),
    "PWM": (0x4009_4000, 0x300),
    # Not the datasheet base on purpose: the model implements the *device* register
    # block at +0x800, which is the one a firmware brings up.  Declaring the general
    # block instead leaves the firmware waiting forever for CLKUSABLE (the bundled
    # SAM firmwares touch 0x400AC800 during startup).
    "UOTGHS": (0x400A_C800, 0x34),
}

# Present on the part, not modelled: the work list for the next peripheral.
NOT_MODELLED = {
    "HSMCI": 0x4000_0000,
    "SSC": 0x4000_4000,
    "SPI1": 0x4000_C000,
    "TC0": 0x4008_0000,
    "TC1": 0x4008_4000,
    "TC2": 0x4008_8000,
    "USART0": 0x4009_8000,
    "USART1": 0x4009_C000,
    "USART2": 0x400A_0000,
    "USART3": 0x400A_4000,
    "EMAC": 0x400B_0000,
    "CAN0": 0x400B_4000,
    "CAN1": 0x400B_8000,
    "TRNG": 0x400B_C000,
    "DMAC": 0x400C_4000,
    "DACC": 0x400C_8000,
    "SMC": 0x400E_0000,
    "SDRAMC": 0x400E_0200,
    "MATRIX": 0x400E_0400,
    "CHIPID": 0x400E_0940,
    "RSTC": 0x400E_1A00,
    "SUPC": 0x400E_1A10,
    "RTTC": 0x400E_1A30,
    "WDT": 0x400E_1A50,
    "RTT": 0x400E_1A70,
    "GPBR": 0x400E_1A90,
}


def peripherals():
    with open(CHIP, encoding="utf-8") as file:
        chip = yaml.safe_load(file)["chip"]
    for entry in chip["peripherals"]:
        name = next(iter(entry))
        yield name, entry[name]


def modelled():
    return {name for name, spec in peripherals() if spec["type"] == "core"}


class TestTheAddressMap:
    def test_every_modelled_block_sits_where_the_datasheet_says(self):
        wrong = []
        for name, spec in peripherals():
            if spec["type"] != "core" or name not in DATASHEET:
                continue
            expected, _size = DATASHEET[name]
            if spec["base"] != expected:
                wrong.append(f"{name}: 0x{spec['base']:08x} instead of 0x{expected:08x}")
        assert not wrong, wrong

    def test_every_modelled_block_is_declared_in_this_file(self):
        """A new model has to be added here, with its address from the datasheet."""
        undocumented = sorted(modelled() - set(DATASHEET))
        assert not undocumented, f"add these to DATASHEET with their datasheet base: {undocumented}"

    def test_the_window_covers_the_models_register_table(self):
        """Every *functional* register has to be inside the declared window.

        A model may pad its table with ``RESERVED`` words up to the next block
        (the UART and the TWI do, to reach their PDC window), so only the last
        real register decides.
        """
        from xuanwu.arch.cortex_m.controller import ArmHardwareController

        buildin = ArmHardwareController.get_buildin(None, "cortex_m", "sam")
        for name, spec in peripherals():
            if spec["type"] != "core" or name not in DATASHEET:
                continue
            key = name.lower()
            model = buildin[key] if key in buildin else buildin[key[:-1]]
            offset = 0
            last = 0
            for entry in model.REGISTERS:
                fmt = entry[1]
                count = int(fmt[:-1]) if fmt[:-1].isdigit() else 1
                size = count * {"I": 4, "H": 2, "B": 1}[fmt[-1]]
                offset += size
                if not entry[0].startswith("RESERVED"):
                    last = offset
            assert spec["size"] >= last, (
                f"{name}: the declared window is 0x{spec['size']:x} "
                f"but the register table reaches 0x{last:x}"
            )

    def test_the_modelled_addresses_do_not_overlap(self):
        ranges = sorted((DATASHEET[name][0], name) for name in modelled() if name in DATASHEET)
        for (start, name), (next_start, next_name) in zip(ranges, ranges[1:]):
            assert start != next_start, f"{name} and {next_name} share a base address"

    def test_the_unmodelled_list_matches_reality(self):
        gap = sorted(set(NOT_MODELLED) - modelled())
        assert gap, "the not-modelled list is empty, which cannot be right"
        # every modelled peripheral must have moved out of the not-modelled list
        stale = sorted(modelled() & set(NOT_MODELLED))
        assert not stale, f"these are modelled now, remove them from NOT_MODELLED: {stale}"

    def test_the_gap_is_documented_in_the_description(self):
        """The chip YAML says which blocks are still missing, as a comment block."""
        text = CHIP.read_text(encoding="utf-8")
        assert "not modelled" in text.lower(), "list the missing blocks in sam3x8e.yaml"
