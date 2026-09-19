# -*- coding: utf-8 -*-

"""Shared fixtures for the xuanwu test suite."""

import shutil
import struct
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

# Chip descriptions ship inside the package so installed copies can find them.
CHIP_DIR = SRC_DIR / "xuanwu" / "data" / "chips" / "arm" / "cortex_m"

# Test firmware lives outside the package.
FIRMWARE_DIRS = (REPO_ROOT / "tests" / "firmware",)

CHIPS = {
    "stm32f411": CHIP_DIR / "stm32f411.yaml",
    "sam3x8e": CHIP_DIR / "sam3x8e.yaml",
}

# Load address ranges used to pick the matching chip description for a firmware.
CHIP_BY_LOAD_ADDRESS = (
    (0x08000000, 0x08100000, "stm32f411"),
    (0x00080000, 0x00100000, "sam3x8e"),
)

# SAM UART/SPI models need a socat pty bridge.
HAS_SOCAT = shutil.which("socat") is not None

requires_socat = pytest.mark.skipif(not HAS_SOCAT, reason="socat is required by the SAM UART/SPI models")


def elf_load_address(path: Path) -> int:
    """Lowest PT_LOAD physical address -- identifies which chip a firmware targets."""
    data = path.read_bytes()
    (phoff,) = struct.unpack_from("<I", data, 28)
    (phentsize,) = struct.unpack_from("<H", data, 42)
    (phnum,) = struct.unpack_from("<H", data, 44)
    bases = []
    for index in range(phnum):
        offset = phoff + index * phentsize
        p_type, _off, _vaddr, paddr, filesz, _memsz = struct.unpack_from("<IIIIII", data, offset)
        if p_type == 1 and filesz:  # PT_LOAD
            bases.append(paddr)
    if not bases:
        raise ValueError(f"{path} has no loadable segments")
    return min(bases)


def chip_for_firmware(path: Path) -> str:
    base = elf_load_address(path)
    for low, high, name in CHIP_BY_LOAD_ADDRESS:
        if low <= base < high:
            return name
    raise ValueError(f"no chip description matches load address 0x{base:08x} of {path.name}")


def iter_firmware_files():
    for directory in FIRMWARE_DIRS:
        if directory.is_dir():
            yield from sorted(directory.rglob("*.elf"))


def discover_firmwares():
    """Return [(firmware_path, chip_name)] for every test firmware we can find."""
    found = []
    for path in iter_firmware_files():
        try:
            found.append((path, chip_for_firmware(path)))
        except ValueError:
            continue
    return found


def discover_chips():
    """Return every chip description shipped in the package."""
    return sorted(CHIP_DIR.glob("*.yaml"))


def pytest_generate_tests(metafunc):
    """Parametrize tests that ask for a `firmware` or `chip_yaml` fixture."""
    if "firmware" in metafunc.fixturenames:
        cases = discover_firmwares()
        metafunc.parametrize("firmware", cases, ids=[f"{path.parent.name}/{path.name}" for path, _chip in cases])
    if "chip_yaml" in metafunc.fixturenames:
        cases = discover_chips()
        metafunc.parametrize("chip_yaml", cases, ids=[path.name for path in cases])


def find_firmware(name: str) -> Path:
    for directory in FIRMWARE_DIRS:
        if not directory.is_dir():
            continue
        for candidate in sorted(directory.rglob(name)):
            return candidate
    pytest.skip(f"test firmware {name} not found under {[str(d) for d in FIRMWARE_DIRS]}")


@pytest.fixture(scope="session")
def chip_paths():
    """Mapping of chip name -> chip description path."""
    return dict(CHIPS)


@pytest.fixture(scope="session")
def stm32f411_path() -> Path:
    return CHIPS["stm32f411"]


@pytest.fixture(scope="session")
def sam3x8e_path() -> Path:
    return CHIPS["sam3x8e"]


@pytest.fixture(scope="session")
def stm32f411_firmware() -> Path:
    """A firmware that needs no serial bridge, so it runs on any platform."""
    return find_firmware("Blink_m4.ino.elf")


@pytest.fixture(scope="session")
def sam3x8e_firmware() -> Path:
    """A SAM3X firmware that prints over its UART (needs the socat bridge)."""
    return find_firmware("Blink_uart_m3.ino.elf")


@pytest.fixture(scope="session")
def stm32f411_with_firmware(stm32f411_path, stm32f411_firmware):
    from xuanwu import XuanWu

    device = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    device.reset()
    return device
