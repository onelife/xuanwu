# -*- coding: utf-8 -*-

"""Cortex-M core peripherals and the exception/interrupt engine."""

from .constants import CCR, CFSR, CONTROL, CSR, EPSR, ICSR, SHCSR, Exception_
from .controller import ArmHardwareController
from .cp import ArmHardwareCp
from .dbg import ArmHardwareDbg
from .dwt import ArmHardwareDwt
from .nvic import ArmHardwareNvic
from .scb import ArmHardwareScb
from .scid import ArmHardwareScid
from .systick import ArmHardwareSystick

__all__ = [
    "Exception_",
    "CONTROL",
    "EPSR",
    "ICSR",
    "CCR",
    "CFSR",
    "SHCSR",
    "CSR",
    "ArmHardwareController",
    "ArmHardwareScid",
    "ArmHardwareSystick",
    "ArmHardwareNvic",
    "ArmHardwareScb",
    "ArmHardwareCp",
    "ArmHardwareDbg",
    "ArmHardwareDwt",
    "CORE_PERIPHERALS",
]


CORE_PERIPHERALS = {
    "scid": ArmHardwareScid,
    "systick": ArmHardwareSystick,
    "nvic": ArmHardwareNvic,
    "scb": ArmHardwareScb,
    "cp": ArmHardwareCp,
    "dbg": ArmHardwareDbg,
    "dwt": ArmHardwareDwt,
}
"""Built-in Cortex-M peripherals, keyed by the lower-cased name used in the chip YAML."""
