# -*- coding: utf-8 -*-

"""The firmware tree and ``tests/firmware/board.yaml`` must agree.

Every image under ``tests/firmware/`` has to live under a directory that
``board.yaml`` declares, otherwise it silently drops out of the parametrised smoke
tests -- which is easy to miss, because a firmware that nothing runs looks exactly
like a firmware that passes.

The configuration is load-bearing rather than cosmetic: two unrelated parts load at
0x08000000 (STM32F411 and STM32F767), so the load address alone cannot say which
chip an image belongs to.  That is why ``board.yaml`` wins over the address map.
"""

from pathlib import Path

import pytest
import yaml

pytestmark = pytest.mark.conformance

FIRMWARE_DIR = Path(__file__).resolve().parents[1] / "firmware"
CONFIG = FIRMWARE_DIR / "board.yaml"
CHIP_DIR = Path(__file__).resolve().parents[2] / "src" / "xuanwu" / "data" / "chips" / "arm" / "cortex_m"


def boards() -> list:
    with open(CONFIG, encoding="utf-8") as file:
        return yaml.safe_load(file)["boards"]


def chip_names() -> set:
    return {path.stem for path in CHIP_DIR.glob("*.yaml")}


class TestBoardMatrix:
    def test_every_entry_names_a_sketch_that_exists(self):
        for board in boards():
            sketch = FIRMWARE_DIR / board["sketch"] / f"{board['sketch']}.ino"
            assert sketch.is_file(), f"{board['name']}: {sketch} does not exist"

    def test_a_sketch_is_named_after_its_directory(self):
        # The Arduino CLI only builds `X/X.ino`, so a mismatch cannot build at all.
        for board in boards():
            sketch = FIRMWARE_DIR / board["sketch"] / f"{board['sketch']}.ino"
            if sketch.is_file():
                assert sketch.parent.name == sketch.stem

    def test_every_entry_has_a_fqbn_and_a_board_name(self):
        for board in boards():
            assert board.get("fqbn") and ":" in board["fqbn"], f"{board['name']}: no usable fqbn"
            assert board.get("board"), f"{board['name']}: no board description"

    def test_a_declared_chip_has_a_description(self):
        known = chip_names()
        for board in boards():
            chip = board.get("chip")
            if chip:
                assert chip in known, f"{board['name']}: chip {chip!r} has no description (known: {sorted(known)})"

    def test_output_directories_are_unique(self):
        outputs = [board["output"] for board in boards()]
        assert len(outputs) == len(set(outputs)), f"two boards share an output directory: {outputs}"

    def test_a_board_without_a_chip_says_what_is_missing(self):
        # Building an image for a part nobody has described yet is deliberate: it
        # is the work list.  Recording why makes that explicit instead of looking
        # like an oversight.
        for board in boards():
            if board.get("chip"):
                continue
            assert board.get("note"), f"{board['name']}: no chip description yet, so it needs a 'note'"


class TestEveryImageIsAccountedFor:
    def test_no_firmware_is_silently_dropped(self):
        declared = {board["output"] for board in boards()}
        unaccounted = [
            path
            for path in sorted(FIRMWARE_DIR.rglob("*.elf"))
            if not any(parent.name in declared for parent in path.parents)
        ]
        assert not unaccounted, (
            "these images live outside every output directory in board.yaml, so no test "
            f"would ever run them: {[str(p.relative_to(FIRMWARE_DIR)) for p in unaccounted]}"
        )

    def test_a_board_with_a_chip_has_a_built_image(self):
        for board in boards():
            if not board.get("chip"):
                continue
            assert list((FIRMWARE_DIR / board["output"]).glob("*.elf")), f"{board['name']}: nothing built"

    def test_every_board_has_a_built_image(self):
        for board in boards():
            assert list((FIRMWARE_DIR / board["output"]).glob("*.elf")), f"{board['name']}: nothing built"

    def test_the_suite_runs_every_board_that_has_a_chip(self):
        from conftest import discover_firmwares

        runnable = {board["output"] for board in boards() if board.get("chip")}
        picked = {path.parent.name for path, _chip in discover_firmwares()}
        assert runnable <= picked, f"{sorted(runnable - picked)} is built and described but never parametrised"

    def test_a_board_without_a_chip_is_not_run(self):
        from conftest import discover_firmwares

        unrunnable = {board["output"] for board in boards() if not board.get("chip")}
        picked = {path.parent.name for path, _chip in discover_firmwares()}
        assert not (unrunnable & picked), (
            f"{sorted(unrunnable & picked)} has no chip description but was parametrised anyway"
        )

    def test_every_image_is_a_loadable_elf(self):
        """Including the ones nothing runs yet: a corrupt artifact must not hide.

        A firmware for a part with no chip description is never executed by a test,
        so this is the only thing standing between a broken build and a committed
        binary that looks fine.
        """
        from conftest import elf_load_address

        for path in sorted(FIRMWARE_DIR.rglob("*.elf")):
            relative = path.relative_to(FIRMWARE_DIR)
            assert path.read_bytes()[:4] == b"\x7fELF", f"{relative} is not an ELF"
            base = elf_load_address(path)  # raises if there is no loadable segment
            assert base % 4 == 0, f"{relative} loads at an unaligned address 0x{base:08x}"
            entry = int.from_bytes(path.read_bytes()[24:28], "little")
            assert entry != 0, f"{relative} has a zero entry point"
