# -*- coding: utf-8 -*-

"""Conformance checks for chip description files.

Adding a new MCU should be a YAML-only change; these tests make malformed or
inconsistent descriptions fail in seconds instead of at simulation time.

A description that builds on another one with ``include:`` is checked *merged*, which is
what the simulator runs: a board overlay that forgot to inherit the part's peripherals
would otherwise pass these checks vacuously.
"""

import pytest

from xuanwu.arch.cortex_m import ArmHardwareController
from xuanwu.chips import load_chip_document
from xuanwu.xuanwu import ARCH_MAPPING, MODE_MAPPING

pytestmark = pytest.mark.conformance

MAPPED_TYPES = {"memory", "peripheral", "remap", "bitband_memory", "bitband_peripheral"}
ALL_TYPES = MAPPED_TYPES | {"core"}

BITBAND_WORDS = 32  # 32 bits per word on the 32-bit parts modelled so far


def load(chip_yaml):
    doc = load_chip_document(str(chip_yaml))
    assert "chip" in doc, f"{chip_yaml.name} has no 'chip' root key"
    return doc["chip"]


def peripherals(chip):
    for entry in chip.get("peripherals", []):
        name = next(iter(entry))
        yield name, entry[name]


def mapped_ranges(chip):
    """Address ranges that actually get mem_map'd/mmio_map'd at runtime."""
    ranges = []
    for name, spec in peripherals(chip):
        kind = spec["type"]
        if kind in ("memory", "peripheral", "bitband_peripheral"):
            ranges.append((spec["base"], spec["base"] + spec["size"], name))
        elif kind == "remap":
            ranges.append((spec["alias"], spec["alias"] + spec["size"], name))
        elif kind == "bitband_memory":
            ranges.append((spec["alias"], spec["alias"] + spec["size"] * BITBAND_WORDS, name))
        if kind == "bitband_peripheral":
            ranges.append((spec["alias"], spec["alias"] + spec["size"] * BITBAND_WORDS, name))
    return ranges


def core_ranges(chip):
    for name, spec in peripherals(chip):
        if spec["type"] == "core":
            yield (spec["base"], spec["base"] + spec["size"], name)


def buildin_for(chip):
    family = chip["name"][:3].lower()
    return ArmHardwareController.get_buildin(None, chip.get("mode"), family)


class TestChipDescriptionSchema:
    def test_arch_and_mode_are_known(self, chip_yaml):
        chip = load(chip_yaml)
        assert chip.get("arch") in ARCH_MAPPING, f"unknown arch {chip.get('arch')!r}"
        assert chip.get("mode") in MODE_MAPPING.get(chip["arch"], {}), f"unknown mode {chip.get('mode')!r}"

    def test_chip_name_dispatches_to_a_known_family(self, chip_yaml):
        chip = load(chip_yaml)
        # The hardware controller picks the vendor registry from name[:3].
        try:
            buildin_for(chip)
        except Exception as err:  # noqa: BLE001
            pytest.fail(f"cannot resolve a vendor registry for {chip['name']!r}: {err}")

    def test_every_peripheral_has_a_known_type(self, chip_yaml):
        chip = load(chip_yaml)
        for name, spec in peripherals(chip):
            assert spec["type"] in ALL_TYPES, f"{name}: unknown type {spec['type']!r}"
            for key in ("base", "size"):
                assert key in spec, f"{name}: missing '{key}'"
            if spec["type"] in ("remap", "bitband_memory", "bitband_peripheral"):
                assert "alias" in spec, f"{name}: type {spec['type']} needs an 'alias'"


class TestChipDescriptionConsistency:
    def test_mapped_regions_do_not_overlap(self, chip_yaml):
        chip = load(chip_yaml)
        ranges = sorted(mapped_ranges(chip))
        for (start, end, name), (next_start, next_end, next_name) in zip(ranges, ranges[1:]):
            assert end <= next_start, f"{name} [0x{start:08x},0x{end:08x}) overlaps {next_name} from 0x{next_start:08x}"

    def test_core_peripherals_live_inside_a_mapped_region(self, chip_yaml):
        chip = load(chip_yaml)
        # `core` peripherals are registered as IO inside an enclosing
        # `peripheral` / `bitband_peripheral` region.
        containers = [r for r in mapped_ranges(chip) if r[0] is not None]
        for start, end, name in core_ranges(chip):
            inside = any(lo <= start and end <= hi for lo, hi, _ in containers)
            assert inside, f"{name} [0x{start:08x},0x{end:08x}) is not inside any mapped peripheral region"

    def test_boot_address_is_mapped(self, chip_yaml):
        chip = load(chip_yaml)
        boot = chip.get("boot", 0)
        if not boot:
            return
        ranges = mapped_ranges(chip)
        assert any(lo <= boot < hi for lo, hi, _ in ranges), f"boot 0x{boot:08x} is not inside a mapped region"

    def test_core_peripheral_names_resolve_to_a_model(self, chip_yaml):
        chip = load(chip_yaml)
        buildin = buildin_for(chip)
        for name, _spec in peripherals(chip):
            if _spec["type"] != "core":
                continue
            key = name.lower()
            if key not in buildin:
                key = key[:-1]  # indexed peripherals: GPIOA -> gpio
            assert key in buildin, (
                f"{name}: no model found (tried {name.lower()!r} and {key!r}); "
                f"known models: {sorted(buildin)}"
            )


class TestDmaWindows:
    """A `core` peripheral may declare a second register window: its DMA block.

    On the SAM3X that block sits 0x100 bytes after the peripheral's own registers
    -- the UART's PDC is at 0x400E0900 while the UART block the model implements
    ends at 0x400E0824 -- so it is *not* inside the peripheral's own block.  What
    has to hold is that the window is mapped and non-empty, and that it does not
    overlap the peripheral's own block: the controller registers a separate IO
    record for it, and overlapping IO windows make an access ambiguous, which the
    memory controller rejects at run time.
    """

    def dma_windows(self, chip):
        for name, spec in peripherals(chip):
            if "dma_base" not in spec:
                continue
            start = spec["dma_base"]
            yield name, spec, start, start + spec.get("dma_size", 0)

    def test_every_dma_window_is_mapped(self, chip_yaml):
        chip = load(chip_yaml)
        containers = mapped_ranges(chip)
        for name, _spec, start, end in self.dma_windows(chip):
            assert end > start, f"{name}: the dma window is empty"
            assert any(lo <= start and end <= hi for lo, hi, _ in containers), (
                f"{name}: dma window [0x{start:08x},0x{end:08x}) is not inside any mapped region"
            )

    def test_every_dma_window_has_a_size(self, chip_yaml):
        chip = load(chip_yaml)
        for name, spec, _start, _end in self.dma_windows(chip):
            assert spec.get("dma_size", 0) > 0, f"{name}: 'dma_base' needs a 'dma_size'"

    def test_the_dma_window_does_not_overlap_its_peripheral(self, chip_yaml):
        chip = load(chip_yaml)
        for name, spec, start, end in self.dma_windows(chip):
            block_start = spec["base"]
            block_end = block_start + spec["size"]
            assert end <= block_start or start >= block_end, (
                f"{name}: dma window [0x{start:08x},0x{end:08x}) overlaps the peripheral block "
                f"[0x{block_start:08x},0x{block_end:08x}); accesses inside the overlap would be ambiguous"
            )
