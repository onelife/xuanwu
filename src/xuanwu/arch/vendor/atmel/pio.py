# -*- coding: utf-8 -*-

"""Parallel I/O controller (PIOA..PIOF).

This is an *adapter*: the register layout, the write-one-to-set / write-one-to-clear
pairs and the write-protection key live here; the level bookkeeping and the device
notifications live in :class:`xuanwu.peripherals.gpio.GpioPort`.
"""

from typing import Any, Callable, Tuple, Union

from ....config import logger
from ....peripherals.gpio import GpioPort
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamGpio"]


class ArmSamGpio(ArmHardwareBase):
    """General-purpose I/Os"""

    NAME = "GPIO"
    REGISTERS = (
        ("PER", "I", 0xFFFFFFFF),
        ("PDR", "I", 0xFFFFFFFF),
        ("PSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("OER", "I", 0xFFFFFFFF),
        ("ODR", "I", 0xFFFFFFFF),
        ("OSR", "I", 0x00000000),
        ("RESERVED1", "I", 0x00000000),
        ("IFER", "I", 0xFFFFFFFF),
        ("IFDR", "I", 0xFFFFFFFF),
        ("IFSR", "I", 0x00000000),
        ("RESERVED2", "I", 0x00000000),
        ("SODR", "I", 0xFFFFFFFF),
        ("CODR", "I", 0xFFFFFFFF),
        ("ODSR", "I", 0xFFFFFFFF),
        ("PDSR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("ISR", "I", 0x00000000),
        ("MDER", "I", 0xFFFFFFFF),
        ("MDDR", "I", 0xFFFFFFFF),
        ("MDSR", "I", 0x00000000),
        ("RESERVED3", "I", 0x00000000),
        ("PUDR", "I", 0xFFFFFFFF),
        ("PUER", "I", 0xFFFFFFFF),
        ("PUSR", "I", 0x00000000),
        ("RESERVED4", "I", 0x00000000),
        ("ABSR", "I", 0xFFFFFFFF),
        ("RESERVED5", "3I", 0x00000000),
        ("SCIFSR", "I", 0xFFFFFFFF),
        ("DIFSR", "I", 0xFFFFFFFF),
        ("IFDGSR", "I", 0x00000000),
        ("SCDR", "I", 0x00003FFF),
        ("RESERVED6", "4I", 0x00000000),
        ("OWER", "I", 0xFFFFFFFF),
        ("OWDR", "I", 0xFFFFFFFF),
        ("OWSR", "I", 0x00000000),
        ("RESERVED7", "I", 0x00000000),
        ("AIMER", "I", 0xFFFFFFFF),
        ("AIMDR", "I", 0xFFFFFFFF),
        ("AIMMR", "I", 0x00000000),
        ("RESERVED8", "I", 0x00000000),
        ("ESR", "I", 0xFFFFFFFF),
        ("LSR", "I", 0xFFFFFFFF),
        ("ELSR", "I", 0x00000000),
        ("RESERVED9", "I", 0x00000000),
        ("FELLSR", "I", 0xFFFFFFFF),
        ("REHLSR", "I", 0xFFFFFFFF),
        ("FRLHSR", "I", 0x00000000),
        ("RESERVED10", "I", 0x00000000),
        ("LOCKSR", "I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
    )

    # The write-one-to-set / write-one-to-clear pairs, and what each side means.
    SET_PAIRS = {
        "PER": ("selection", True),
        "OER": ("directions", True),
        "PUER": ("pullups", True),
        "OWER": ("opens", True),
        "MDER": ("multi_drive", True),
        "IFER": ("input_filter", True),
    }
    CLEAR_PAIRS = {
        "PDR": ("selection", False),
        "ODR": ("directions", False),
        "PUDR": ("pullups", False),
        "OWDR": ("opens", False),
        "MDDR": ("multi_drive", False),
        "IFDR": ("input_filter", False),
    }

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.NAME = self._name.upper()
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        self.port = GpioPort(self.NAME, width=32)

    # -- what the device layer subscribes to -------------------------------

    def add_hook(self, pin: int, fn: Tuple[Union[Callable, None]]) -> None:
        self.port.add_hook(pin, fn)

    def remove_hook(self, pin: int, fn: Tuple[Union[Callable, None]]) -> None:
        self.port.remove_hook(pin, fn)

    def reset(self):
        super().reset()
        self.port.reset()
        for name in (
            "PSR", "OSR", "IFSR", "ODSR", "PDSR", "IMR", "ISR", "MDSR", "PUSR",
            "ABSR", "IFDGSR", "SCDR", "OWSR", "AIMMR", "ELSR", "FRLHSR", "LOCKSR",
            "WPMR", "WPSR",
        ):
            self.write_register(name, 0x0)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "ISR":
            self.write_register(name, 0)
        elif name == "PDSR":
            # What the pins are actually at, including anything a device drives.
            # PSR stays the stored mirror of PER: it says which pins the PIO
            # peripheral controls, not the level they are at.
            data = self.port.levels()
            self.write_register("PDSR", data)
        elif name == "ODSR":
            data = self.port.outputs
            self.write_register("ODSR", data)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in self.SET_PAIRS or name in self.CLEAR_PAIRS:
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                logger.warning(f"[{name_:16s}]: Ignore write")
                return data_orig
            attribute, enabled = (self.SET_PAIRS if name in self.SET_PAIRS else self.CLEAR_PAIRS)[name]
            reg = name[:-2] + "S" + name[-1:]
            val = self.read_register(reg)
            new_val = (val | data) if name in self.SET_PAIRS else (val & ~data)
            self.write_register(reg, new_val)
            setattr(self.port, attribute, new_val & 0xFFFFFFFF)
            data = 0
        elif name == "ABSR":
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
            else:
                self.port.peripheral_function = data
        elif name == "SODR":
            self.port.set_output_bits(data)
            self.write_register("ODSR", self.port.outputs)
            self.write_register("PDSR", self.port.levels())
            data = 0
        elif name == "CODR":
            self.port.clear_output_bits(data)
            self.write_register("ODSR", self.port.outputs)
            self.write_register("PDSR", self.port.levels())
            data = 0
        elif name == "ODSR":
            owsr = self.read_register("OWSR")
            masked = (data & owsr) | (data_orig & ~owsr)
            self.port.set_outputs(masked)
            self.write_register("PDSR", self.port.levels())
            data = masked
        elif name in ["IER", "AIMER"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name in ["IDR", "AIMDR"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
        elif name == "DIFSR":
            reg = "IFDGSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "SCIFSR":
            reg = "IFDGSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "LSR":
            reg = "ELSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "ESR":
            reg = "ELSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "REHLSR":
            reg = "FRLHSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "FELLSR":
            reg = "FRLHSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x50494F00:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data
