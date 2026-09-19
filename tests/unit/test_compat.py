# -*- coding: utf-8 -*-

"""The module split must not break existing imports.

``xuanwu.arch.armv7m``, ``xuanwu.arch.atmel_sam`` and ``xuanwu.arch.stm_stm``
are kept as deprecated aliases for one release.
"""

import importlib
import warnings

import pytest

LEGACY_MODULES = [
    ("xuanwu.arch.armv7m", "ArmHardwareController"),
    ("xuanwu.arch.atmel_sam", "ArmSamUart"),
    ("xuanwu.arch.stm_stm", "ArmStmRcc"),
]


@pytest.mark.parametrize("module_name,attribute", LEGACY_MODULES)
def test_legacy_import_path_still_works(module_name, attribute):
    module = importlib.import_module(module_name)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        module = importlib.reload(module)
    assert getattr(module, attribute) is not None
    assert any(issubclass(warning.category, DeprecationWarning) for warning in caught), (
        f"{module_name} should warn about the move"
    )


def test_new_module_paths_expose_the_expected_registries():
    from xuanwu.arch.base import ArmHardwareBase, Register, arm_core_registers
    from xuanwu.arch.cortex_m import CORE_PERIPHERALS, ArmHardwareController
    from xuanwu.arch.vendor.atmel import BUILDIN as ATMEL_BUILDIN
    from xuanwu.arch.vendor.st import BUILDIN as ST_BUILDIN

    assert ArmHardwareBase is not None and Register is not None
    assert "r0" in arm_core_registers
    assert ArmHardwareController is not None
    assert set(CORE_PERIPHERALS) == {"scid", "systick", "nvic", "scb", "cp", "dbg", "dwt", "fpu"}
    assert set(ATMEL_BUILDIN) == {"pmc", "dma", "efc", "gpio", "adc", "pwm", "uart", "spi", "twi", "uotghs"}
    assert set(ST_BUILDIN) == {"rcc", "gpio"}


def test_vendor_models_live_in_one_file_each():
    """The 1655-line atmel_sam.py must not come back."""
    from pathlib import Path

    from xuanwu.arch.vendor import atmel

    directory = Path(atmel.__file__).resolve().parent
    for name in ("pmc", "pdc", "pio", "adc", "uart", "spi", "pwm", "uotghs"):
        assert (directory / f"{name}.py").is_file(), f"missing vendor/atmel/{name}.py"
