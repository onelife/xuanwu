# -*- coding: utf-8 -*-

from typing import Union, Any

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS

from .exception import XwInvalidParameter


__all__ = ["RegisterController"]


class RegisterController(object):
    """Register control unit"""

    def __init__(self, box: Uc, arch: int, mode: int, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._box = box
        if arch == UC_ARCH_ARM:
            from .arch import arm_core_registers

            self._str2num = arm_core_registers
        else:
            raise XwInvalidParameter("Unsupported architecture: {arch}")
        self._t_mode = 0x1 if mode & UC_MODE_MCLASS else 0x0

    def __getattr__(self, name: str) -> Any:
        if "_str2num" in self.__dict__ and name in self.__dict__["_str2num"]:
            return self._box.reg_read(self._str2num[name.lower()])
        else:
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{name}'")

    # def __setattr__(self, name: str, value: Any) -> None:
    #     if "_str2num" in self.__dict__ and name in self.__dict__["_str2num"]:
    #         return self._box.reg_write(self._str2num[name.lower()], value)
    #     else:
    #         self.__dict__[name] = value

    @property
    def pc_t(self):
        return self.pc | self._t_mode

    def write(self, register: Union[str, int], value: int) -> None:
        if isinstance(register, str):
            register = self._str2num[register.lower()]
        return self._box.reg_write(register, value)

    def read(self, register: Union[str, int]) -> int:
        if isinstance(register, str):
            register = self._str2num[register.lower()]
        return self._box.reg_read(register)
