# -*- coding: utf-8 -*-

"""Coprocessor access control (Cortex-M4 FPU)."""

from typing import Any

from ..base import ArmHardwareBase

__all__ = ["ArmHardwareCp"]


    # def write(self, address: int, size: int, data: int, internal: Optional[bool] = False) -> None:
    #     super().write(address, size, data, internal)


class ArmHardwareCp(ArmHardwareBase):
    """Coprocessor of Cortex-M4"""

    NAME = "CP"
    REGISTERS = (("CPACR", "I", 0x00F00000),)

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        self.write_register("CPACR", 0x0)
