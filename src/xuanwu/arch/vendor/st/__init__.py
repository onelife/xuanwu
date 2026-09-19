# -*- coding: utf-8 -*-

"""STMicroelectronics STM32 peripherals."""

from .gpio import ArmStmGpio
from .rcc import ArmStmRcc

__all__ = ["ArmStmRcc", "ArmStmGpio", "BUILDIN"]


BUILDIN = {
    "rcc": ArmStmRcc,
    "gpio": ArmStmGpio,
}
"""Peripheral models keyed by the name used in the chip YAML."""
