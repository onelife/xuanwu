# -*- coding: utf-8 -*-

"""Cortex-M core peripherals and the exception/interrupt engine."""

from .constants import CCR, CFSR, CONTROL, CSR, EPSR, ICSR, SHCSR, Exception_
from .controller import FP_FRAME_SIZE, ArmHardwareController
from .cp import ArmHardwareCp
from .dbg import ArmHardwareDbg
from .dwt import ArmHardwareDwt
from .fpu import ArmHardwareFpu
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
    "ArmHardwareFpu",
    "CORE_PERIPHERALS",
    "FP_FRAME_SIZE",
]


CORE_PERIPHERALS = {
    "scid": ArmHardwareScid,
    "systick": ArmHardwareSystick,
    "nvic": ArmHardwareNvic,
    "scb": ArmHardwareScb,
    "cp": ArmHardwareCp,
    "dbg": ArmHardwareDbg,
    "dwt": ArmHardwareDwt,
    "fpu": ArmHardwareFpu,
}
"""Built-in Cortex-M peripherals, keyed by the lower-cased name used in the chip YAML.

``fpu`` is only declared by chips that actually have a floating-point unit, so
its presence in ``hw.perif`` is also what tells the GDB stub to serve the VFP
target description.
"""
