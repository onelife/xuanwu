# -*- coding: utf-8 -*-

"""Foundation for the Arm peripheral models.

``Register`` describes one register inside a peripheral block, and
``ArmHardwareBase`` turns a table of those descriptions into a readable/writable
register file with per-access fix hooks.
"""

from abc import ABC, abstractmethod
from collections import OrderedDict, namedtuple
from struct import Struct
from typing import TYPE_CHECKING, Any, Optional

from unicorn import Uc
from unicorn import arm_const as uc_arm

from ..config import logger
from ..exception import XwInvalidMemoryAddress, XwInvalidMemorySize

if TYPE_CHECKING:  # pragma: no cover - typing only, avoids an import cycle
    from .cortex_m.controller import ArmHardwareController

__all__ = ["arm_core_registers", "arm_context_registers", "Register", "IrqOp", "ArmHardwareBase"]


arm_core_registers = {
    # Arm core
    "r0": uc_arm.UC_ARM_REG_R0,
    "r1": uc_arm.UC_ARM_REG_R1,
    "r2": uc_arm.UC_ARM_REG_R2,
    "r3": uc_arm.UC_ARM_REG_R3,
    "r4": uc_arm.UC_ARM_REG_R4,
    "r5": uc_arm.UC_ARM_REG_R5,
    "r6": uc_arm.UC_ARM_REG_R6,
    "r7": uc_arm.UC_ARM_REG_R7,
    "r8": uc_arm.UC_ARM_REG_R8,
    "r9": uc_arm.UC_ARM_REG_R9,
    "r10": uc_arm.UC_ARM_REG_R10,
    "r11": uc_arm.UC_ARM_REG_R11,
    "r12": uc_arm.UC_ARM_REG_R12,
    "sp": uc_arm.UC_ARM_REG_SP,  # stack pointer
    "lr": uc_arm.UC_ARM_REG_LR,  # link register
    "pc": uc_arm.UC_ARM_REG_PC,  # program counter
    # Armv7-M
    "msp": uc_arm.UC_ARM_REG_MSP,  # main stack pointer
    "psp": uc_arm.UC_ARM_REG_PSP,  # process stack pointer
    # Armv7
    "xpsr": uc_arm.UC_ARM_REG_XPSR,  # special-purpose program status register
    "xpsr_nzcvqg": uc_arm.UC_ARM_REG_XPSR_NZCVQG,  # special-purpose program status register with all APSR bits (default value without GE)
    "apsr": uc_arm.UC_ARM_REG_APSR,  # application program status register
    "ipsr": uc_arm.UC_ARM_REG_IPSR,  # interrupt program status register
    "epsr": uc_arm.UC_ARM_REG_EPSR,  # execution program status register
    # Armv7-M
    "primask": uc_arm.UC_ARM_REG_PRIMASK,  # exception mask register
    "faultmask": uc_arm.UC_ARM_REG_FAULTMASK,  # fault mask
    "basepri": uc_arm.UC_ARM_REG_BASEPRI,  # base priority mask
    # ?
    "control": uc_arm.UC_ARM_REG_CONTROL,  # special-purpose control register
    # FP extension: S0-S31 alias D0-D15 which alias Q0-Q15
    "fpscr": uc_arm.UC_ARM_REG_FPSCR,
}

# Added in a loop because the names are numbered; the FP register file is part of
# the CPU model, so it is always accessible even on a core without an FPU.
arm_core_registers.update({f"s{index}": getattr(uc_arm, f"UC_ARM_REG_S{index}") for index in range(32)})
arm_core_registers.update({f"d{index}": getattr(uc_arm, f"UC_ARM_REG_D{index}") for index in range(16)})
arm_core_registers.update({f"q{index}": getattr(uc_arm, f"UC_ARM_REG_Q{index}") for index in range(16)})


arm_context_registers = ["r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr"]


Register = namedtuple("Register", ["format", "offset", "mask"])


IrqOp = namedtuple("IrqOp", ["is_enabled", "set_pending", "set_active", "get_priority"])


class ArmHardwareBase(ABC):
    """Arm (32-bit) peripheral base class"""

    NAME = "BASE"
    ENDIAN = "<"
    REGISTERS = (
        # (NAME, FORMAT, WRITE_MASK)
    )

    def _remap_read_callback(self, box: Uc, offset: int, size: int, address: int) -> bytearray:
        return int.from_bytes(box.mem_read(address + offset, size), self._endian_str)

    def _remap_write_callback(self, box: Uc, offset: int, size: int, data: bytes, address: int) -> None:
        return box.mem_write(address + offset, data)

    def __init__(self, box: Uc, ctl: "ArmHardwareController", name: str, base: int, size: int, **kwargs: Any) -> None:
        # super().__init__(**kwargs)
        self._endian_str = "little"
        self._box = box
        self._ctl = ctl
        self._name = name
        self._base = base & 0xFFFFFFFF
        self._fix_after_read: callable = None
        self._fix_before_write: callable = None
        # initialize registers
        offset = 0
        registers = OrderedDict()
        for (k, *v) in self.REGISTERS:
            format_ = Struct(self.ENDIAN + v[0])
            registers[k] = Register(format_, offset, v[1])
            offset += format_.size
        self.registers = registers
        self.values = bytearray(Struct(f"{self.ENDIAN}{offset}B").pack(*([0] * offset)))

    def __getattr__(self, name: str) -> Any:
        if "registers" in self.__dict__ and name in self.__dict__["registers"]:
            return self.read_register(name)
        else:
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{name}'")

    def __setattr__(self, name: str, value: Any) -> None:
        if "registers" in self.__dict__ and name in self.__dict__["registers"]:
            return self.write_register(name, value)
        # Honour data descriptors (properties) declared on the class.  Writing
        # straight into __dict__ would shadow them, so a property setter would
        # silently never run -- which is how `peripheral.bridge = device` used to
        # leave the old bridge in place.
        descriptor = getattr(type(self), name, None)
        if hasattr(descriptor, "__set__"):
            return descriptor.__set__(self, value)
        self.__dict__[name] = value

    @abstractmethod
    def reset(self):
        logger.debug(f"{self.NAME} memory size: {hex(sum([reg.format.size for reg in self.registers.values()]))}")

    def read(self, address: int, size: int, internal: Optional[bool] = False) -> int:
        offset = address - self._base
        data_orig = None
        for name, record in self.registers.items():
            format_, offset_, mask = record
            if offset >= offset_ + format_.size:
                continue
            byte_offset = offset & 0x3
            offset &= ~0x3
            if offset != offset_:
                raise XwInvalidMemoryAddress(f"Invalid address to read {self.NAME}: 0x{address:08X} ({size})")
            if byte_offset + size > format_.size:
                raise XwInvalidMemorySize(f"Invalid size to read {self.NAME}: 0x{address:08X} ({size})")
            byte_mask = (1 << (size * 8)) - 1
            name_ = ".".join([self.NAME, name])
            # logger.debug(f"[{name_:16s}] (R0): 0x{address:08x}{f' ({size})' if size != 4 else ''}")
            data_orig = format_.unpack(self.values[offset_ : offset_ + format_.size])[0]
            break
        if data_orig is None:
            raise XwInvalidMemoryAddress(f"Invalid address to read {self.NAME}: 0x{address:08X} ({size})")
        data = (data_orig >> (byte_offset * 8)) & byte_mask
        if self._fix_after_read:
            data = self._fix_after_read(name, record, data)
        if not internal:
            logger.debug(f"[{name_:16s}] (R): 0x{address:08x}{f' ({size})' if size != 4 else ''} => 0x{data:08x}")
        return data

    def write(self, address: int, size: int, data: int, internal: Optional[bool] = False) -> None:
        offset = address - self._base
        data_orig = None
        for name, record in self.registers.items():
            format_, offset_, mask = record
            if offset >= offset_ + format_.size:
                continue
            byte_offset = offset & 0x3
            offset &= ~0x3
            if offset != offset_:
                raise XwInvalidMemoryAddress(f"Invalid address to write {self.NAME}: 0x{address:08X} ({size})")
            if byte_offset + size > format_.size:
                raise XwInvalidMemorySize(f"Invalid size to write {self.NAME}: 0x{address:08X} ({size})")
            byte_mask = (1 << (size * 8)) - 1
            name_ = ".".join([self.NAME, name])
            # logger.debug(f"[{name_:16s}] (W0): 0x{self._base + offset:08x}{f' ({size})' if size != 4 else ''} <= 0x{data:08x}")
            data_orig = format_.unpack(self.values[offset_ : offset_ + format_.size])[0]
            break
        if data_orig is None:
            raise XwInvalidMemoryAddress(f"Invalid address to write {self.NAME}: 0x{address:08X} ({size})")
        data = (data_orig & ~mask) | (((data & byte_mask) << (byte_offset * 8)) & mask)
        if self._fix_before_write:
            data = self._fix_before_write(name, record, data, data_orig)
        value = format_.pack(data)
        self.values[:] = self.values[:offset] + value + self.values[offset + size :]
        if not internal:
            logger.debug(
                f"[{name_:16s}] (W): 0x{self._base + offset:08x}{f' ({size})' if size != 4 else ''} <= 0x{data:08x}"
            )

    def read_register(self, name: str) -> int:
        # logger.debug(f"{self.NAME} read: {name}")
        format_, offset, _ = self.registers[name.upper()]
        return format_.unpack(self.values[offset : offset + format_.size])[0]

    def write_register(self, name: str, data: int) -> None:
        # if name not in ["CVR", "CSR"]:
        #     logger.debug(f"{self.NAME} write: {name}, 0x{data:08x}")
        format_, offset, _ = self.registers[name.upper()]
        self.values[:] = self.values[:offset] + format_.pack(data) + self.values[offset + format_.size :]
