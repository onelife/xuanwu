# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
from collections import OrderedDict, namedtuple
from struct import Struct
from enum import IntEnum
from functools import partial
import os
from typing import Optional, Union, Any, Dict, Tuple, Set

from unicorn import Uc, UC_HOOK_CODE, UC_HOOK_INTR
from unicorn.arm_const import *
from serial import Serial

from ..config import logger, EXCP
from ..register import RegisterController
from ..memory import MemoryController
from ..exception import XwUnknownHardware, XwInvalidMemoryAddress, XwInvalidMemorySize


__all__ = ["arm_core_registers", "arm_context_registers", "ArmHardwareController"]

arm_core_registers = {
    # Arm core
    "r0": UC_ARM_REG_R0,
    "r1": UC_ARM_REG_R1,
    "r2": UC_ARM_REG_R2,
    "r3": UC_ARM_REG_R3,
    "r4": UC_ARM_REG_R4,
    "r5": UC_ARM_REG_R5,
    "r6": UC_ARM_REG_R6,
    "r7": UC_ARM_REG_R7,
    "r8": UC_ARM_REG_R8,
    "r9": UC_ARM_REG_R9,
    "r10": UC_ARM_REG_R10,
    "r11": UC_ARM_REG_R11,
    "r12": UC_ARM_REG_R12,
    "sp": UC_ARM_REG_SP,  # stack pointer
    "lr": UC_ARM_REG_LR,  # link register
    "pc": UC_ARM_REG_PC,  # program counter
    # Armv7-M
    "msp": UC_ARM_REG_MSP,  # main stack pointer
    "psp": UC_ARM_REG_PSP,  # process stack pointer
    # Armv7
    "xpsr": UC_ARM_REG_XPSR,  # special-purpose program status register
    "xpsr_nzcvqg": UC_ARM_REG_XPSR_NZCVQG,  # special-purpose program status register with all APSR bits (default value without GE)
    "apsr": UC_ARM_REG_APSR,  # application program status register
    "ipsr": UC_ARM_REG_IPSR,  # interrupt program status register
    "epsr": UC_ARM_REG_EPSR,  # execution program status register
    # Armv7-M
    "primask": UC_ARM_REG_PRIMASK,  # exception mask register
    "faultmask": UC_ARM_REG_FAULTMASK,  # fault mask
    "basepri": UC_ARM_REG_BASEPRI,  # base priority mask
    # ?
    "control": UC_ARM_REG_CONTROL,  # special-purpose control register
    # FP extension
    # S0-31, D0-15
    "fpscr": UC_ARM_REG_FPSCR,
}

arm_context_registers = ["r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr"]

HARDWARE_MAPPING = {}

Register = namedtuple("Register", ["format", "offset", "mask"])
IrqOp = namedtuple("IrqOp", ["is_enabled", "set_pending", "set_active", "get_priority"])


class Exception_(IntEnum):
    Reset = 1
    NMI = 2
    HardFault = 3
    MemManage = 4
    BusFault = 5
    UsageFault = 6
    SVCall = 11
    DebugMonitor = 12
    PendSV = 14
    SysTick = 15


class CONTROL(IntEnum):
    SPSEL = 1


class ArmHardwareController(object):
    """Armv7-m hardware control unit"""

    def __init__(self, box: Uc, reg: RegisterController, mem: MemoryController, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._box = box
        self._reg = reg
        self._mem = mem
        self.format_ = Struct("<I")
        self.perif: Dict[str, object] = {}
        self._irq_gp = 0
        self._irq_op: Dict[int, IrqOp] = {}
        self._irq_pending: List[int] = []
        self._irq_handling: List[int] = []
        self._box.hook_add(UC_HOOK_INTR, self.system_interrupt_callback)

    def register_irq_op(self, irq: int, irq_op: IrqOp) -> None:
        self._irq_op[irq] = irq_op

    def register_irq_gp(self, gp: int) -> None:
        self._irq_gp = gp

    def set_irq_pending(self, irq: int) -> None:
        if irq not in self._irq_pending:
            # logger.debug(f"Interrupt: Add IRQ{irq} in pending queue")
            self._irq_pending.append(irq)

    def clear_irq_pending(self, irq: int) -> None:
        if irq in self._irq_pending:
            logger.debug(f"Interrupt: Remove IRQ{irq} from pending queue")
            self._irq_pending.remove(irq)

    def push_stack(self, value: int) -> int:
        sp = self._reg.read("sp")
        sp -= 4
        self._reg.write("sp", sp)
        self._mem.write(sp, self.format_.pack(value))
        return sp

    def pop_stack(self) -> int:
        sp = self._reg.read("sp")
        data = self._mem.read(sp, 4)
        sp += 4
        self._reg.write("sp", sp)
        return self.format_.unpack(data)[0]

    def push_context(self) -> None:
        sp = self._reg.read("sp")
        for reg in reversed(arm_context_registers):
            sp -= 4
            self._mem.write(sp, self.format_.pack(self._reg.read(reg)))
        self._reg.write("sp", sp)

    def pop_context(self) -> None:
        for reg in arm_context_registers:
            if reg == "xpsr":
                reg = "xpsr_nzcvqg"
            self._reg.write(reg, self.pop_stack())

    def jump_isr(self, irq: int) -> None:
        # get handler address
        vector = self.perif["scb"].read_register("VTOR")
        exp = irq + 16
        offset = vector | (exp << 2)
        addr = self.format_.unpack(self._mem.read(offset, 4))[0]
        # logger.debug(f"IRQ{irq} handler @0x{addr:08x}")
        # check if exception
        ipsr = self._reg.read("ipsr")
        if ipsr == 0:
            control = self._reg.read("control")
            if control & (1 << CONTROL.SPSEL):
                # Return to Thread mode, exception return uses non-floating-point state from the PSP and
                # execution uses PSP after return.
                exc_return = 0xFFFFFFFD
            else:
                # Return to Thread mode, exception return uses non-floating-point state from MSP and
                # execution uses MSP after return.
                exc_return = 0xFFFFFFF9
        else:
            # Return to Handler mode, exception return uses non-floating-point state from the MSP and
            # execution uses MSP after return.
            exc_return = 0xFFFFFFF1
        # set registers
        self._reg.write("ipsr", exp)
        self._reg.write("pc", addr)
        self._reg.write("lr", exc_return)

    def get_next_irq(self) -> Tuple[Union[bool, int]]:
        gp_mask = ~((1 << (self._irq_gp + 1)) - 1) & 0xFF
        pending = [
            (self._irq_op[irq].get_priority() & gp_mask, self._irq_op[irq].get_priority(), irq)
            for irq in self._irq_pending
        ]
        pending.sort()
        faultmask = self._reg.read("faultmask") & 0x01
        primask = self._reg.read("primask") & 0x01
        basepri = self._reg.read("basepri") & 0xFF
        next_irq = None
        # find the next active IRQ
        while pending:
            if next_irq is not None:
                # drop from pending queue
                self._irq_pending.remove(next_irq)
                self._irq_op[next_irq].set_pending(state=False)
            next_gp, next_pri, next_irq = pending.pop(0)
            if faultmask and next_irq > (Exception_.NMI - 16):
                continue
            if primask and next_irq > (Exception_.HardFault - 16):
                continue
            if basepri and next_pri >= basepri:
                continue
            if self._irq_op[next_irq].is_enabled():
                break
        else:
            if next_irq is not None:
                # drop from pending queue
                self._irq_pending.remove(next_irq)
                self._irq_op[next_irq].set_pending(state=False)
            return (False, None, None)
        is_preempted = False
        # check if preempt
        if not self._irq_handling:
            is_preempted = True
            cur_irq = None
        else:
            handling = [
                (self._irq_op[irq].get_priority() & gp_mask, self._irq_op[irq].get_priority(), irq)
                for irq in self._irq_handling
            ]
            cur_gp, _, cur_irq = handling.pop(0)
            is_preempted = (cur_gp > next_gp) and (next_irq not in self._irq_handling)
        return (is_preempted, next_irq, cur_irq)

    def system_interrupt_callback(self, box: Uc, intno: int, data: Any):
        if intno == EXCP.EXCEPTION_EXIT:
            exp = self._reg.read("ipsr")
            if exp == 0:
                logger.error(f"ipsr == {exp} when handling EXCEPTION_EXIT!?")
                raise RuntimeError(f"ipsr == {exp} when handling EXCEPTION_EXIT!?")
            irq = exp - 16
            # logger.debug(f"Exit IRQ{irq} handler...")
            # clear active
            self._irq_op[irq].set_active(state=False)
            if irq not in self._irq_handling:
                logger.error(f"Completed IRQ{irq} not in handling list!?")
                raise RuntimeError(f"Completed IRQ{irq} not in handling list!?")
            else:
                self._irq_handling.remove(irq)
            # tail-chaining
            is_preempted, next_irq, cur_irq = self.get_next_irq()
            if not is_preempted:
                next_irq = cur_irq
            if not next_irq:
                # restore registers
                self._reg.write("ipsr", 0)
                pc = self._reg.read("pc")
                control = self._reg.read("control")
                # select sp
                if pc & 0x04:
                    control |= 1 << CONTROL.SPSEL
                else:
                    control &= ~(1 << CONTROL.SPSEL)
                self._reg.write("control", control)
                # restore context
                # logger.debug(f"Restore context after IRQ{next_irq}")
                self.pop_context()
            else:
                self._irq_handling.append(next_irq)
                self._irq_pending.remove(next_irq)
                self._irq_op[next_irq].set_pending(state=False)
                # set active
                self._irq_op[next_irq].set_active(state=True)
                # jump to ISR
                # logger.debug(f"Enter IRQ{next_irq} handler...")
                self.jump_isr(next_irq)
        else:
            pc = self._reg.read("pc")
            ipsr = self._reg.read("ipsr")
            logger.info(f"intno: {intno}, data: {data}, pc: {pc:08x}, ipsr: {ipsr:08x}")
            raise

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        if not self._irq_pending:
            return
        is_preempted, next_irq, _ = self.get_next_irq()
        if not is_preempted:
            return
        # preempt
        ipsr = self._reg.read("ipsr")
        if ipsr == 0:
            if self._irq_handling:
                logger.error(f"IRQ handling list is not empty in thread mode!? {self._irq_handling}")
                raise RuntimeError(f"IRQ handling list is not empty in thread mode! {self._irq_handling}")
            # save context
            # logger.debug(f"Save context before IRQ{next_irq}")
            self.push_context()
        self._irq_handling.append(next_irq)
        self._irq_pending.remove(next_irq)
        self._irq_op[next_irq].set_pending(state=False)
        # set active
        self._irq_op[next_irq].set_active(state=True)
        # jump to ISR
        # logger.debug(f"Enter IRQ{next_irq} handler...")
        self.jump_isr(next_irq)

    def map_memory(self, chip: Dict[str, Any]) -> None:
        mode = chip.get("mode")
        name = chip["name"][:3].lower()
        buildin = HARDWARE_MAPPING.get(mode)
        if not buildin:
            return
        for k in buildin.keys():
            if isinstance(buildin[k], dict):
                buildin[k] = buildin[k][name]
        # IRQ process function
        self._box.hook_add(UC_HOOK_CODE, self.system_clock_callback)
        # do remap boot address
        boot = chip.get("boot", 0x0)
        if boot != 0x0:
            self._mem.remap(boot, 0x0)
        # do mapping
        for device in chip.get("peripherals", []):
            name = list(device.keys())[0]
            if device[name]["type"] == "core":
                name_ = name.lower()
                class_ = name_
                if class_ not in buildin:
                    class_ = class_[:-1]
                    if class_ not in buildin:
                        raise XwUnknownHardware(f"Unknown core peripheral: {name}")
                base = device[name]["base"]
                size = device[name]["size"]
                # create instance
                buildin_ = buildin[class_](self._box, self, name, **device[name])
                self.perif[name_] = buildin_
                self._mem.register_io(base, size, self.perif[name_].read, self.perif[name_].write, name)
                if hasattr(buildin_, "system_clock_callback"):
                    logger.debug(f"[{name:8s}]: Add system_clock_callback")
                    self._box.hook_add(UC_HOOK_CODE, buildin_.system_clock_callback)

    def reset(self):
        for name, buildin in self.perif.items():
            logger.debug(f"Reset {name}")
            buildin.reset()


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

    def __init__(self, box: Uc, ctl: ArmHardwareController, name: str, base: int, size: int, **kwargs: Any) -> None:
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
        else:
            self.__dict__[name] = value

    @abstractmethod
    def reset(self):
        logger.debug(f"{self.NAME} memory size: {hex(sum([reg.format.size for reg in self.registers.values()]))}")

    def read(self, address: int, size: int, internal: Optional[bool] = False) -> int:
        offset = address - self._base
        data = None
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
        value = None
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
        # value = data & mask
        self.values[:] = self.values[:offset] + format_.pack(data) + self.values[offset + format_.size :]


class ArmHardwareScid(ArmHardwareBase):
    """system control and ID registers not in the SCB"""

    NAME = "SCID"
    REGISTERS = (
        ("RESERVED0", "I", 0x00000000),
        ("ICTR", "I", 0x00000000),
        ("ACTLR", "I", 0x00000307),
        ("RESERVED1", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._interrupt_lines = kwargs.get("interrupt_lines", 7)

    def reset(self):
        super().reset()
        self.write_register("ICTR", self._interrupt_lines)
        self.write_register("ACTLR", 0x0)


# # Software Trigger Interrupt Register
# (("STIR", "I", 0x000001FF),)


class CSR(IntEnum):
    ENABLE = 0
    TICKINT = 1
    CLKSOURCE = 2
    COUNTFLAG = 16


class ArmHardwareSystick(ArmHardwareBase):
    """System timer"""
    # TODO: CLK selection?

    NAME = "SYSTICK"
    REGISTERS = (
        ("CSR", "I", 0x00000007),
        ("RVR", "I", 0x00FFFFFF),
        ("CVR", "I", 0x00FFFFFF),
        ("CALIB", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._step = kwargs.get("step", 1)  # count down step
        self._cvr = 0
        self._rvr = 0
        self._irq = -1
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CSR", 0x0)
        self.write_register("RVR", 0x0)
        self.write_register("CVR", 0x0)
        # The SysTick calibration value is fixed to 10500, which gives a reference time base of 1 ms
        # with the SysTick clock set to 10.5 MHz (HCLK/8, with HCLK set to 84 MHz).
        self.write_register("CALIB", 0x00002904)
        self._csr = 0
        self._cvr = 0
        self._rvr = 0

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        # csr = self.read_register("CSR")
        if self._csr & (1 << CSR.ENABLE) == 0:
            return
        # cvr = self.read_register("CVR")
        self._cvr -= self._step
        if self._cvr <= 0:
            self._cvr = self._rvr
            # cvr = self.read_register("RVR")
            self._csr |= 1 << CSR.COUNTFLAG
            # trigger interrupt
            if self._csr & (1 << CSR.TICKINT):
                self._ctl.set_irq_pending(self._irq)
            self.write_register("CSR", self._csr)
        # self.write_register("CVR", self._cvr)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        name_ = ".".join([self.NAME, name])
        # csr = self.read_register("CSR")
        if name == "CSR":
            data = self._csr
            # read to clear
            self._csr &= ~(1 << CSR.COUNTFLAG)
            self.write_register("CSR", self._csr)
            logger.debug(f"[{name_:16s}]: Clear CSR.COUNTFLAG")
        elif name == "CVR":
            data = self._cvr
            self.write_register("CVR", self._cvr)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        # csr = self.read_register("CSR")
        if name == "CSR":
            self._csr = data
            # TODO: CLKSOURCE, If no external clock is provided, this bit reads as 1 and ignores writes.
            pass
        elif name == "CVR":
            # set status, if selected clock
            self._cvr = 0
            data = 0
            self._csr &= ~(1 << CSR.COUNTFLAG)
            self.write_register("CSR", self._csr)
            logger.debug(f"[{name_:16s}]: Clear CVR and CSR.COUNTFLAG")
        elif name == "RVR":
            self._rvr = data
        return data


class ArmHardwareNvic(ArmHardwareBase):
    """nested vectored interrupt controller"""

    NAME = "NVIC"
    REGISTERS = tuple()
    REGISTERS_TEMPLATE = (
        # Interrupt Set-enable Registers
        ("ISER{0}", "I", 0xFFFFFFFF),
        ("RESERVED0", "{0}I", 0x00000000),
        # Interrupt Clear-enable Registers
        ("ICER{0}", "I", 0xFFFFFFFF),
        ("RESERVED1", "{0}I", 0x00000000),
        # Interrupt Set-pending Registers
        ("ISPR{0}", "I", 0xFFFFFFFF),
        ("RESERVED2", "{0}I", 0x00000000),
        # Interrupt Clear-pending Registers
        ("ICPR{0}", "I", 0x00000000),
        ("RESERVED3", "{0}I", 0x00000000),
        # Interrupt Active Bit Registers
        ("IABR{0}", "I", 0x00000000),
        ("RESERVED4", "{0}I", 0x00000000),
        ("IPR{0}", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        line_num = kwargs.get("interrupt_lines", 7) + 1
        priority_bits = kwargs.get("priority_bits", 4)
        # fix IPR mask
        mask_ = ~((1 << priority_bits) - 1) & 0xFF
        mask = 0
        for _ in range(4):
            mask = (mask << 8) | mask_
        REGS = []
        for name, fmt, mask in ArmHardwareNvic.REGISTERS_TEMPLATE:
            if name.startswith("RESERVED"):
                REGS.extend([(name, fmt.format(32 - line_num), mask)])
            elif name.startswith("IPR"):
                REGS.extend([(name.format(x), fmt, mask) for x in range((line_num * 32 - 16) // 4)])
            else:
                REGS.extend([(name.format(x), fmt, mask) for x in range(line_num)])
        ArmHardwareNvic.REGISTERS = tuple(REGS)
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        for i in range(240):
            self._ctl.register_irq_op(
                i,
                IrqOp(
                    partial(self.is_enabled, irq=i),
                    partial(self.set_pending, irq=i),
                    partial(self.set_active, irq=i),
                    partial(self.get_priority, irq=i),
                ),
            )

    def reset(self):
        super().reset()

    def _reg2irq(reg_val: int, reg_num: int) -> Set[int]:
        irqs = set()
        for i in range(32):
            if reg_val & 0x01:
                irqs.add(32 * reg_num + i)
            reg_val >>= 1
        return irqs

    def _irq2reg(irq: int, to_ipr: Optional[bool] = False) -> Tuple[int]:
        if not to_ipr:
            offset = irq % 32
            reg_num = irq // 32
        else:
            offset = (irq % 4) * 8
            reg_num = irq // 4
        return (offset, reg_num)

    def is_enabled(self, irq: int) -> bool:
        if irq < 0 or irq >= 240:
            logger.error(f"[{self.NAME:8s}]: Invalid IRQ when get status, {irq}")
            raise RuntimeError(f"Invalid IRQ when get status, {irq}")
        offset, reg_num = self._irq2reg(irq)
        iser = self.read_register(f"ISER{reg_num}")
        return iser & (1 << offset) != 0

    def set_pending(self, irq: int, state: Optional[bool] = True) -> None:
        if irq < 0 or irq >= 240:
            logger.error(f"[{self.NAME:8s}]: Invalid IRQ when set pending, {irq}")
            raise RuntimeError(f"Invalid IRQ when set pending, {irq}")
        offset, reg_num = self._irq2reg(irq)
        ispr = self.read_register(f"ISPR{reg_num}")
        if state:
            ispr &= ~(1 << offset)
        else:
            ispr |= 1 << offset
        self.write_register(f"ISPR{reg_num}", ispr)

    def set_active(self, irq: int, state: Optional[bool] = False) -> None:
        if irq < 0 or irq >= 240:
            logger.error(f"[{self.NAME:8s}]: Invalid IRQ when set active, {irq}")
            raise RuntimeError(f"Invalid IRQ when set active, {irq}")
        offset, reg_num = self._irq2reg(irq)
        iabr = self.read_register(f"IABR{reg_num}")
        if state:
            iabr &= ~(1 << offset)
        else:
            iabr |= 1 << offset
        self.write_register(f"IABR{reg_num}", iabr)

    def get_priority(self, irq: int) -> int:
        if irq < 0 or irq >= 240:
            logger.error(f"[{self.NAME:8s}]: Invalid IRQ when get priority, {irq}")
            raise RuntimeError(f"Invalid IRQ when get priority, {irq}")
        offset, reg_num = self._irq2reg(irq, True)
        ipr = self.read_register(f"IPR{reg_num}")
        return (ipr >> offset) & 0xFF

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name.startswith("ICER") or name.startswith("ICPR"):
            name_ = str(name)
            name_[1] = "S"
            data = self.read_register(name_)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name.startswith("ISER") or name.startswith("ISPR"):
            data = data_orig | data
            if name.startswith("ISPR"):
                set_bits = (data ^ data_orig) & data
                irqs = self._reg2irq(set_bits, int(name[-1]))
                for irq in irqs:
                    self._ctl.set_irq_pending(irq)
        elif name.startswith("ICER") or name.startswith("ICPR"):
            name_ = str(name)
            name_[1] = "S"
            data_orig_ = self.read_register(name_)
            data_ = data_orig_ & ~data
            self.write_register(name_, data_)
            if name.startswith("ICPR"):
                clear_bits = (~data ^ data_orig_) & data
                irqs = self._reg2irq(clear_bits, int(name[-1]))
                for irq in irqs:
                    self._ctl.clear_irq_pending(irq)
            data = 0
        elif name.startswith("IABR"):
            logger.warning(f"[{name_:16s}]: {name} is read-only")
        elif name == "STIR":
            # When the USERSETMPEND bit in the SCR is set to 1, unprivileged software can access
            logger.debug(f"[{name_:16s}]: Software trigger IRQ{data}")
        return data


class ICSR(IntEnum):
    PENDSTCLR = 25
    PENDSTSET = 26
    PENDSVCLR = 27
    PENDSVSET = 28
    NMIPENDSET = 31


class SHCSR(IntEnum):
    MEMFAULTACT = 0
    BUSFAULTACT = 1
    USGFAULTACT = 3
    SVCALLACT = 7
    MONITORACT = 8
    PENDSVACT = 10
    SYSTICKACT = 11
    USGFAULTPENDED = 12
    MEMFAULTPENDED = 13
    BUSFAULTPENDED = 14
    SVCALLPENDED = 15
    MEMFAULTENA = 16
    BUSFAULTENA = 17
    USGFAULTENA = 18


class ArmHardwareScb(ArmHardwareBase):
    """system control block"""

    ENABLE_BIT = {
        Exception_.MemManage: SHCSR.MEMFAULTENA,
        Exception_.BusFault: SHCSR.BUSFAULTENA,
        Exception_.UsageFault: SHCSR.USGFAULTENA,
    }

    PENDING_BIT = {
        # ICSR
        Exception_.NMI: ICSR.NMIPENDSET,
        Exception_.PendSV: ICSR.PENDSVSET,
        Exception_.SysTick: ICSR.PENDSTSET,
        # SHCSR
        Exception_.MemManage: SHCSR.MEMFAULTPENDED,
        Exception_.BusFault: SHCSR.BUSFAULTPENDED,
        Exception_.UsageFault: SHCSR.USGFAULTPENDED,
        Exception_.SVCall: SHCSR.SVCALLPENDED,
    }

    ACTIVE_BIT = {
        Exception_.MemManage: SHCSR.MEMFAULTACT,
        Exception_.BusFault: SHCSR.BUSFAULTACT,
        Exception_.UsageFault: SHCSR.USGFAULTACT,
        Exception_.SVCall: SHCSR.SVCALLACT,
        Exception_.DebugMonitor: SHCSR.MONITORACT,
        Exception_.PendSV: SHCSR.PENDSVACT,
        Exception_.SysTick: SHCSR.SYSTICKACT,
    }

    NAME = "SCB"
    REGISTERS = (
        ("CPUID", "I", 0x00000000),
        ("ICSR", "I", 0x9E000000),
        ("VTOR", "I", 0xFFFFFF80),  # vary from devices
        ("AIRCR", "I", 0x0FFF0707),
        ("SCR", "I", 0x00000016),
        ("CCR", "I", 0x0000031B),
        ("SHPR1", "I", 0x00FFFFFF),
        ("SHPR2", "I", 0xFF000000),
        ("SHPR3", "I", 0xFFFF0000),
        ("SHCSR", "I", 0x0007FD8B),
        ("CFSR", "I", 0x030FBFBB),
        ("HFSR", "I", 0xC0000002),
        ("DFSR", "I", 0x00000000),
        ("MMFAR", "I", 0xFFFFFFFF),
        ("BFSR", "I", 0xFFFFFFFF),
        ("AFSR", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._cpuid = kwargs.get("cpuid", 0x410FC241)
        self._fix_before_write = self.fix_before_write
        # NMI
        self._ctl.register_irq_op(
            Exception_.NMI - 16,
            IrqOp(
                lambda: True,
                partial(self.set_pending, exp=Exception_.NMI),
                lambda: None,
                partial(self.get_priority, exp=Exception_.NMI),
            ),
        )
        # HardFault
        self._ctl.register_irq_op(
            Exception_.HardFault - 16,
            IrqOp(
                lambda: True,
                lambda: None,
                lambda: None,
                partial(self.get_priority, exp=Exception_.HardFault),
            ),
        )
        # MemManage
        self._ctl.register_irq_op(
            Exception_.MemManage - 16,
            IrqOp(
                partial(self.is_enabled, exp=Exception_.MemManage),
                partial(self.set_pending, exp=Exception_.MemManage),
                partial(self.set_active, exp=Exception_.MemManage),
                partial(self.get_priority, exp=Exception_.MemManage),
            ),
        )
        # BusFault
        self._ctl.register_irq_op(
            Exception_.BusFault - 16,
            IrqOp(
                partial(self.is_enabled, exp=Exception_.BusFault),
                partial(self.set_pending, exp=Exception_.BusFault),
                partial(self.set_active, exp=Exception_.BusFault),
                partial(self.get_priority, exp=Exception_.BusFault),
            ),
        )
        # UsageFault
        self._ctl.register_irq_op(
            Exception_.UsageFault - 16,
            IrqOp(
                partial(self.is_enabled, exp=Exception_.UsageFault),
                partial(self.set_pending, exp=Exception_.UsageFault),
                partial(self.set_active, exp=Exception_.UsageFault),
                partial(self.get_priority, exp=Exception_.UsageFault),
            ),
        )
        # SVCall
        self._ctl.register_irq_op(
            Exception_.SVCall - 16,
            IrqOp(
                lambda: True,
                partial(self.set_pending, exp=Exception_.SVCall),
                partial(self.set_active, exp=Exception_.SVCall),
                partial(self.get_priority, exp=Exception_.SVCall),
            ),
        )
        # DebugMonitor
        self._ctl.register_irq_op(
            Exception_.DebugMonitor - 16,
            IrqOp(
                lambda: True,
                lambda: None,
                partial(self.set_active, exp=Exception_.DebugMonitor),
                partial(self.get_priority, exp=Exception_.DebugMonitor),
            ),
        )
        # PendSV
        self._ctl.register_irq_op(
            Exception_.PendSV - 16,
            IrqOp(
                lambda: True,
                partial(self.set_pending, exp=Exception_.PendSV),
                partial(self.set_active, exp=Exception_.PendSV),
                partial(self.get_priority, exp=Exception_.PendSV),
            ),
        )
        # SysTick
        self._ctl.register_irq_op(
            Exception_.SysTick - 16,
            IrqOp(
                lambda: True,
                partial(self.set_pending, exp=Exception_.SysTick),
                partial(self.set_active, exp=Exception_.SysTick),
                partial(self.get_priority, exp=Exception_.SysTick),
            ),
        )

    def reset(self):
        super().reset()
        self.write_register("CPUID", 0x410FC240)  # TODO: Arm Cortex-M4
        self.write_register("ICSR", 0x0)
        self.write_register("VTOR", 0x0)
        self.write_register("AIRCR", 0xFA050000)
        self.write_register("SCR", 0xFA050000)
        self.write_register("CCR", 0x00000200)
        self.write_register("SHPR1", 0x0)
        self.write_register("SHPR2", 0x0)
        self.write_register("SHPR3", 0x0)
        self.write_register("SHCSR", 0x0)
        self.write_register("CFSR", 0x0)
        self.write_register("HFSR", 0x0)
        self.write_register("DFSR", 0x0)
        self.write_register("AFSR", 0x0)

    def is_enabled(self, exp: int) -> bool:
        if exp not in self.ENABLE_BIT:
            logger.error(f"[{self.NAME:8s}]: Invalid exception when get status, {exp}")
            raise RuntimeError(f"Invalid exception when get status, {exp}")
        offset = self.ENABLE_BIT[exp]
        shcsr = self.read_register("SHCSR")
        return shcsr & (1 << offset) != 0

    def set_pending(self, exp: int, state: Optional[bool] = True) -> None:
        if exp not in self.PENDING_BIT:
            logger.error(f"[{self.NAME:8s}]: Invalid exception when set pending, {exp}")
            raise RuntimeError(f"Invalid exception when set pending, {exp}")
        if exp in (Exception_.NMI, Exception_.PendSV, Exception_.SysTick):
            REG = "ICSR"
        else:
            REG = "SHCSR"
        offset = self.PENDING_BIT[exp]
        val = self.read_register(REG)
        if state:
            val &= ~(1 << offset)
        else:
            val |= 1 << offset
        self.write_register(REG, val)

    def set_active(self, exp: int, state: Optional[bool] = False) -> None:
        if exp not in self.ACTIVE_BIT:
            logger.error(f"[{self.NAME:8s}]: Invalid exception when set active, {exp}")
            raise RuntimeError(f"Invalid exception when set active, {exp}")
        offset = self.ACTIVE_BIT[exp]
        shcsr = self.read_register("SHCSR")
        if state:
            shcsr &= ~(1 << offset)
        else:
            shcsr |= 1 << offset
        self.write_register("SHCSR", shcsr)

    def get_priority(self, exp: int) -> int:
        if exp <= 0 or exp >= 16:
            logger.error(f"[{self.NAME:8s}]: Invalid exception when get priority, {exp}")
            raise RuntimeError(f"Invalid exception when get priority, {exp}")
        if exp == Exception_.Reset:
            return -3
        elif exp == Exception_.NMI:
            return -2
        elif exp == Exception_.HardFault:
            return -1
        reg_num = exp // 4
        offset = (exp % 4) * 8
        shpr = self.read_register(f"SHPR{reg_num}")
        return (shpr >> offset) & 0xFF

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        if name == "AIRCR":
            self._ctl.register_irq_gp((data & 0x00000700) >> 8)
        elif name == "ICSR":
            set_bits = (data ^ data_orig) & data
            if set_bits & (1 << ICSR.PENDSTCLR):
                self._ctl.clear_irq_pending(Exception_.SysTick - 16)
            if set_bits & (1 << ICSR.PENDSVCLR):
                self._ctl.clear_irq_pending(Exception_.PendSV - 16)
            if set_bits & (1 << ICSR.PENDSTSET):
                self._ctl.set_irq_pending(Exception_.SysTick - 16)
            if set_bits & (1 << ICSR.PENDSVSET):
                self._ctl.set_irq_pending(Exception_.PendSV - 16)
            if set_bits & (1 << ICSR.NMIPENDSET):
                self._ctl.set_irq_pending(Exception_.NMI - 16)
            # clear XXXCLR and set XXXSET
            data = (data & 0xF5FFFFFF) | (data_orig & 0x94000000)
        elif name == "SHCSR":
            diff = data ^ data_orig
            mask = 1 << SHCSR.USGFAULTPENDED
            if diff & mask:
                if data & mask:
                    self._ctl.set_irq_pending(Exception_.UsageFault - 16)
                else:
                    self._ctl.clear_irq_pending(Exception_.UsageFault - 16)
            mask = 1 << SHCSR.MEMFAULTPENDED
            if diff & mask:
                if data & mask:
                    self._ctl.set_irq_pending(Exception_.MemManage - 16)
                else:
                    self._ctl.clear_irq_pending(Exception_.MemManage - 16)
            mask = 1 << SHCSR.BUSFAULTPENDED
            if diff & mask:
                if data & mask:
                    self._ctl.set_irq_pending(Exception_.BusFault - 16)
                else:
                    self._ctl.clear_irq_pending(Exception_.BusFault - 16)
            mask = 1 << SHCSR.SVCALLPENDED
            if diff & mask:
                if data & mask:
                    self._ctl.set_irq_pending(Exception_.SVCall - 16)
                else:
                    self._ctl.clear_irq_pending(Exception_.SVCall - 16)
        return data


class ArmHardwareCp(ArmHardwareBase):
    """Coprocessor of Cortex-M4"""

    NAME = "CP"
    REGISTERS = (("CPACR", "I", 0x00F00000),)

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        self.write_register("CPACR", 0x0)


class ArmHardwareDbg(ArmHardwareBase):
    """Debug system"""

    NAME = "DBG"
    REGISTERS = (
        ("DHCSR", "I", 0xFFFF002F),
        ("DCRSR", "I", 0x0001007F),
        ("DCRDR", "I", 0xFFFFFFFF),
        ("DEMCR", "I", 0x010F07F1),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        self.write_register("DHCSR", 0x0)
        self.write_register("DCRSR", 0x0)
        self.write_register("DCRDR", 0x0)
        self.write_register("DEMCR", 0x0)


class ArmHardwareDwt(ArmHardwareBase):
    """data watchpoint trigger"""

    NAME = "DWT"
    REGISTERS = (
        ("CTRL", "I", 0xFF7F1FFF),
        ("CYCCNT", "I", 0xFFFFFFFF),
        ("CPICNT", "I", 0x000000FF),
        ("EXCCNT", "I", 0x000000FF),
        ("SLEEPCNT", "I", 0x000000FF),
        ("LSUCNT", "I", 0x000000FF),
        ("FOLDCNT", "I", 0x000000FF),
        ("PCSR", "I", 0x00000000),
        ("COMP0", "I", 0xFFFFFFFF),
        ("MASK0", "I", 0x0000001F),
        ("FUNCTION0", "I", 0x010FFFAF),
        ("RESERVED0", "I", 0x00000000),
        ("COMP1", "I", 0xFFFFFFFF),
        ("MASK1", "I", 0x0000001F),
        ("FUNCTION1", "I", 0x010FFF2F),
        ("RESERVED1", "I", 0x00000000),
        ("COMP2", "I", 0xFFFFFFFF),
        ("MASK2", "I", 0x0000001F),
        ("FUNCTION2", "I", 0x010FFF2F),
        ("RESERVED2", "I", 0x00000000),
        ("COMP3", "I", 0xFFFFFFFF),
        ("MASK3", "I", 0x0000001F),
        ("FUNCTION3", "I", 0x010FFF2F),
        ("RESERVED3", "989I", 0x00000000),
        ("PID4", "I", 0x00000000),
        ("PID5", "I", 0x00000000),
        ("PID6", "I", 0x00000000),
        ("PID7", "I", 0x00000000),
        ("PID0", "I", 0x00000000),
        ("PID1", "I", 0x00000000),
        ("PID2", "I", 0x00000000),
        ("PID3", "I", 0x00000000),
        ("CID0", "I", 0x00000000),
        ("CID1", "I", 0x00000000),
        ("CID2", "I", 0x00000000),
        ("CID3", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        # unknown reset value
        self.write_register("CTRL", 0x0F000000)


class PID(IntEnum):
    # SUPC = 0
    # RSTC = 1
    RTC = 2
    RTT = 3
    WDG = 4
    PMC = 5
    EEFC0 = 6
    EEFC1 = 7
    UART = 8
    SMC_SDRAMC = 9
    SDRAMC = 10
    PIOA = 11
    PIOB = 12
    PIOC = 13
    PIOD = 14
    PIOE = 15
    PIOF = 16
    USART0 = 17
    USART1 = 18
    USART2 = 19
    USART3 = 20
    HSMCI = 21
    TWI0 = 22
    TWI1 = 23
    SPI0 = 24
    SPI1 = 25
    SSC = 26
    TC0 = 27
    TC1 = 28
    TC2 = 29
    TC3 = 30
    TC4 = 31
    TC5 = 32
    TC6 = 33
    TC7 = 34
    TC8 = 35
    PWM = 36
    ADC = 37
    DACC = 38
    DMAC = 39
    UOTGHS = 40
    TRNG = 41
    EMAC = 42
    CAN0 = 43
    CAN1 = 44


class PMC_MOR(IntEnum):
    MOSCXTEN = 0
    MOSCXTBY = 1
    MOSCRCEN = 3
    MOSCSEL = 24
    CFDEN = 25


class PMC_SR(IntEnum):
    MOSCXTS = 0
    LOCKA = 1
    MCKRDY = 3
    LOCKU = 6
    MOSCSELS = 16
    MOSCRCS = 17
    CFDEV = 18
    CFDS = 19
    FOS = 20


class PMC_PLLAR(IntEnum):
    ONE = 29


class PMC_UCKR(IntEnum):
    UPLLEN = 16


class MCKR_CSS(IntEnum):
    SLOW_CLK = 0
    MAIN_CLK = 1
    PLLA_CLK = 2
    UPLL_CLK = 3


class ArmSamPmc(ArmHardwareBase):
    """Power management controller"""

    NAME = "PMC"
    REGISTERS = (
        ("SCER", "I", 0xFFFFFFFF),
        ("SCDR", "I", 0xFFFFFFFF),
        ("SCSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("PCER0", "I", 0xFFFFFFFC),
        ("PCDR0", "I", 0xFFFFFFFC),
        ("PCSR0", "I", 0x00000000),
        ("UCKR", "I", 0xFFFFFFFF),
        ("MOR", "I", 0xFFFFFFFF),
        ("MCFR", "I", 0x00000000),
        ("PLLAR", "I", 0xFFFFFFFF),
        ("RESERVED1", "I", 0x00000000),
        ("MCKR", "I", 0xFFFFFFFF),
        ("RESERVED2", "I", 0x00000000),
        ("USB", "I", 0xFFFFFFFF),
        ("RESERVED3", "I", 0x00000000),
        ("PCK0", "I", 0xFFFFFFFF),
        ("PCK1", "I", 0xFFFFFFFF),
        ("PCK2", "I", 0xFFFFFFFF),
        ("RESERVED4", "5I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("IMR", "I", 0x00000000),
        ("FSMR", "I", 0xFFFFFFFF),
        ("FSPR", "I", 0xFFFFFFFF),
        ("FOCR", "I", 0xFFFFFFFF),
        ("RESERVED5", "26I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
        ("RESERVED6", "5I", 0x00000000),
        ("PCER1", "I", 0xFFFFFFFF),
        ("PCDR1", "I", 0xFFFFFFFF),
        ("PCSR1", "I", 0x00000000),
        ("PCR", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("SCSR", 0x00000001)
        self.write_register("PCSR0", 0x00000000)
        self.write_register("UCKR", 0x10200800)
        self.write_register("MOR", 0x00000001)
        self.write_register("MCFR", 0x000107D0)  # main: 4M, slow: 32K
        self.write_register("PLLAR", 0x00003F00)
        self.write_register("MCKR", 0x00000001)
        self.write_register("USB", 0x00000000)
        self.write_register("PCK0", 0x00000000)
        self.write_register("PCK1", 0x00000000)
        self.write_register("PCK2", 0x00000000)
        self.write_register("SR", 0x00000008)
        self.write_register("IMR", 0x00000000)
        self.write_register("FSMR", 0x00000000)
        self.write_register("FSPR", 0x00000000)
        self.write_register("WPMR", 0x00000000)
        self.write_register("WPSR", 0x00000000)
        self.write_register("PCSR1", 0x00000000)
        self.write_register("PCR", 0x00000000)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "SR":
            data_ = data
            if data_ & (1 << PMC_SR.MOSCSELS):
                # clear MOSCSELS for next read
                data_ &= ~(1 << PMC_SR.MOSCSELS)
                self.write_register(name, data_)
            if data_ & (1 << PMC_SR.MOSCRCS) == 0:
                mor = self.read_register("MOR")
                if mor & (1 << PMC_MOR.MOSCRCEN):
                    # set MOSCRCEN for next read
                    data_ |= (1 << PMC_SR.MOSCRCS)
                    self.write_register(name, data_)
            if data_ & (1 << PMC_SR.MCKRDY) == 0:
                mckr = self.read_register("MCKR")
                css = mckr & 0x03
                # set MCKRDY for next read
                if css == MCKR_CSS.PLLA_CLK:
                    if data & (1 << PMC_SR.LOCKA):
                        data_ |= (1 << PMC_SR.MCKRDY)
                        self.write_register(name, data_)
                elif css == MCKR_CSS.UPLL_CLK:
                    if data & (1 << PMC_SR.LOCKU):
                        data_ |= (1 << PMC_SR.MCKRDY)
                        self.write_register(name, data_)
                else:
                    data_ |= (1 << PMC_SR.MCKRDY)
                    self.write_register(name, data_)
        # elif name == "IMR":
        #     self.write_register(name, 0)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["SCER", "PCER0", "IER", "PCER1"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                # update status register
                if name == "IER":
                    reg = "SR"
                elif name[-1] in "1234567890":
                    reg = name[:-3] + "S" + name[-2:]
                else:
                    reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                # update PCKRDYx
                if name == "SCER" and (data & 0x00000700):
                    sr = self.read_register("SR")
                    sr |= data & 0x00000700
                    self.write_register("SR", sr)
                elif name.startswith("PCER"):
                    pid = 32 if name == "PCER1" else 0
                    diff = (val ^ new_val) & data
                    for pid_ in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Enable {PID(pid + pid_).name}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["SCDR", "PCDR0", "IDR", "PCDR1"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                # update status register
                reg = name
                if name == "IDR":
                    reg = "SR"
                elif name[-1] in "1234567890":
                    reg = name[:-3] + "S" + name[-2:]
                else:
                    reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                # update PCKRDYx
                if name == "SCDR" and (data & 0x00000700):
                    sr = self.read_register("SR")
                    sr &= ~(data & 0x00000700)
                    self.write_register("SR", sr)
                elif name.startswith("PCER"):
                    pid = 32 if name == "PCER1" else 0
                    diff = (val ^ new_val) & data
                    for pid_ in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable {PID(pid + pid_).name}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["UCKR", "MOR", "PLLAR", "MCKR", "USB", "PCK0", "PCK1", "PCK2", "FSMR", "FSPR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                if name == "MOR":
                    if (data & 0x00FF0000) != 0x00370000:
                        logger.warning(f"[{name_:16s}]: Invalid KEY, 0x{data:08x}")
                        data = data_orig
                    else:
                        data &= ~0x00FF0000
                        sr = self.read_register("SR")
                        if data & (1 << PMC_MOR.MOSCXTEN):
                            data &= ~(1 << PMC_MOR.MOSCXTBY)
                            sr |= 1 << PMC_SR.MOSCXTS
                        else:
                            sr &= ~(1 << PMC_SR.MOSCXTS)
                        if data & (1 << PMC_MOR.MOSCXTBY):
                            sr |= (1 << PMC_SR.MOSCXTS)
                        if data & (1 << PMC_MOR.MOSCRCEN) or (data ^ data_orig) & 0x000000F0:
                            # delay set
                            sr &= ~(1 << PMC_SR.MOSCRCS)
                        if data & (1 << PMC_MOR.MOSCSEL):
                            # delay clear
                            sr |= 1 << PMC_SR.MOSCSELS
                        if data & (1 << PMC_MOR.CFDEN):
                            sr |= 1 << PMC_SR.FOS
                        else:
                            sr &= ~(1 << PMC_SR.FOS)
                        self.write_register("SR", sr)
                elif name == "MCKR":
                    if (data ^ data_orig) & 0x000000FF:
                        sr = self.read_register("SR")
                        # delay set
                        sr &= ~(1 << PMC_SR.MCKRDY)
                        self.write_register("SR", sr)
                elif name == "PLLAR":
                    if data & (1 << PMC_PLLAR.ONE) == 0:
                        logger.warning(f"[{name_:16s}]: Invalid ONE, 0x{data:08x}")
                        data = data_orig
                    else:
                        data &= ~(1 << PMC_PLLAR.ONE)
                    sr = self.read_register("SR")
                    if data & 0xFFFF0000:
                        sr |= 1 << PMC_SR.LOCKA
                    else:
                        sr &= ~(1 << PMC_SR.LOCKA)
                    self.write_register("SR", sr)
                if name == "UCKR":
                    sr = self.read_register("SR")
                    if data & (1 << PMC_UCKR.UPLLEN):
                        sr |= 1 << PMC_SR.LOCKU
                    else:
                        sr &= ~(1 << PMC_SR.LOCKU)
                    self.write_register("SR", sr)
            else:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name in ["FOCR"]:
            data = 0
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x504D4300:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        elif name == "PCR":
            pid = data & 0x3F
            cmd = (data >> 12) & 0x01
            div = (data >> 16) & 0x03
            en = (data >> 28) & 0x01
            if pid not in list(PID):
                logger.warning(f"[{name_:16s}]: Invalid PID, 0x{data:08x}")
            elif cmd and div and pid not in [PID.CAN0, PID.CAN1]:
                logger.warning(f"[{name_:16s}]: {PID(pid).name} doesn't support set DIV, 0x{data:08x}")
            elif cmd:
                # write
                if pid >= 32:
                    reg = "PCSR1"
                    pid_ = pid - 32
                else:
                    reg = "PCSR0"
                    pid_ = pid
                val = self.read_register(reg)
                if en:
                    new_val = val | (1 << pid_)
                else:
                    new_val = val & ~(1 << pid_)
                if val != new_val:
                    logger.debug(f"[{name_:16s}]: {'Enable' if en else 'Disable'} {PID(pid).name}")
                    self.write_register(reg, new_val)
                if div:
                    # TODO: CAN
                    pass
            else:
                # read
                if pid >= 32:
                    reg = "PCSR1"
                    pid_ = pid - 32
                else:
                    reg = "PCSR0"
                    pid_ = pid
                val = self.read_register(reg)
                if val & (1 << pid_):
                    data |= 1 << 28
                else:
                    data &= ~(1 << 28)
                if pid in [PID.CAN0, PID.CAN1]:
                    # TODO: CAN
                    pass
        return data


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

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.NAME = self._name.upper()
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("PSR", 0x0)
        self.write_register("OSR", 0x0)
        self.write_register("IFSR", 0x0)
        self.write_register("ODSR", 0x0)
        self.write_register("PDSR", 0x0)
        self.write_register("IMR", 0x0)
        self.write_register("ISR", 0x0)
        self.write_register("MDSR", 0x0)
        self.write_register("PUSR", 0x0)
        self.write_register("ABSR", 0x0)
        self.write_register("IFDGSR", 0x0)
        self.write_register("SCDR", 0x0)
        self.write_register("OWSR", 0x0)
        self.write_register("AIMMR", 0x0)
        self.write_register("ELSR", 0x0)
        self.write_register("FRLHSR", 0x0)
        self.write_register("LOCKSR", 0x0)
        self.write_register("WPMR", 0x0)
        self.write_register("WPSR", 0x0)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "ISR":
            self.write_register(name, 0)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["PER", "OER", "IFER", "MDER", "PUDR", "OWER"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                if name == "OER":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Enable P{self.NAME[-1]}{i}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["PDR", "ODR", "IFDR", "MDDR", "PUER", "OWDR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                if name == "ODR":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable P{self.NAME[-1]}{i}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name == "ABSR":
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
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
        elif name == "SODR":
            reg = "ODSR"
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (new_val & mask)
            self.write_register("PDSR", pdsr)
            data = 0
            diff = val ^ new_val
            for i in range(32):
                if diff & 0x01:
                    logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                diff >>= 1
        elif name == "CODR":
            reg = "ODSR"
            val = self.read_register(reg)
            new_val = val & ~data
            self.write_register(reg, new_val)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (new_val & mask)
            self.write_register("PDSR", pdsr)
            data = 0
            diff = val ^ new_val
            for i in range(32):
                if diff & 0x01:
                    logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                diff >>= 1
        elif name == "ODSR":
            owsr = self.read_register("OWSR")
            data = (data & owsr) | (data_orig & ~owsr)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (data & mask)
            self.write_register("PDSR", pdsr)
            diff = data ^ data_orig
            data_ = data
            for i in range(32):
                if diff & 0x01:
                    if data_ & 0x01:
                        logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                    else:
                        logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                diff >>= 1
                data_ >>= 1
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


class ADC_CR(IntEnum):
    SWRST = 0
    START = 1


class ADC_MR(IntEnum):
    USEQ = 31


class ADC_ISR(IntEnum):
    DRDY = 24
    GOVRE = 25
    COMPE = 26
    ENDRX = 27
    RXBUFF = 28


class ADC_EMR(IntEnum):
    TAG = 24


class ArmSamAdc(ArmHardwareBase):
    """Analog-to-Digital Converter"""
    # TODO: CLK selection?

    NAME = "ADC"
    REGISTERS = (
        ("CR", "I", 0x00000003),
        ("MR", "I", 0xFFFFFFFF),
        ("SEQR1", "I", 0xFFFFFFFF),
        ("SEQR2", "I", 0xFFFFFFFF),
        ("CHER", "I", 0x0000FFFF),
        ("CHDR", "I", 0x0000FFFF),
        ("CHSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("LCDR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("ISR", "I", 0x00000000),
        ("RESERVED1", "2I", 0x00000000),
        ("OVER", "I", 0x00000000),
        ("EMR", "I", 0xFFFFFFFF),
        ("CWR", "I", 0xFFFFFFFF),
        ("CGR", "I", 0xFFFFFFFF),
        ("COR", "I", 0xFFFFFFFF),
        ("CDR0", "I", 0x00000000),
        ("CDR1", "I", 0x00000000),
        ("CDR2", "I", 0x00000000),
        ("CDR3", "I", 0x00000000),
        ("CDR4", "I", 0x00000000),
        ("CDR5", "I", 0x00000000),
        ("CDR6", "I", 0x00000000),
        ("CDR7", "I", 0x00000000),
        ("CDR8", "I", 0x00000000),
        ("CDR9", "I", 0x00000000),
        ("CDR10", "I", 0x00000000),
        ("CDR11", "I", 0x00000000),
        ("CDR12", "I", 0x00000000),
        ("CDR13", "I", 0x00000000),
        ("CDR14", "I", 0x00000000),
        ("CDR15", "I", 0x00000000),
        ("RESERVED2", "I", 0x00000000),
        ("ACR", "I", 0xFFFFFFFF),
        ("RESERVED3", "19I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CR", 0x0)
        self.write_register("MR", 0x0)
        self.write_register("SEQR1", 0x0)
        self.write_register("SEQR2", 0x0)
        self.write_register("CHSR", 0x0)
        self.write_register("LCDR", 0x0)
        self.write_register("IMR", 0x0)
        self.write_register("ISR", 0x0)
        self.write_register("OVER", 0x0)
        self.write_register("EMR", 0x0)
        self.write_register("CWR", 0x0)
        self.write_register("CGR", 0x0)
        self.write_register("COR", 0x0)
        self.write_register("CDR0", 0x000000)
        self.write_register("CDR1", 0x000101)
        self.write_register("CDR2", 0x000202)
        self.write_register("CDR3", 0x000303)
        self.write_register("CDR4", 0x000404)
        self.write_register("CDR5", 0x000505)
        self.write_register("CDR6", 0x00606)
        self.write_register("CDR7", 0x000707)
        self.write_register("CDR8", 0x000808)
        self.write_register("CDR9", 0x000909)
        self.write_register("CDR10", 0x000A0A)
        self.write_register("CDR11", 0x000B0B)
        self.write_register("CDR12", 0x000C0C)
        self.write_register("CDR13", 0x000D0D)
        self.write_register("CDR14", 0x000E0E)
        self.write_register("CDR15", 0x000F0F)
        self.write_register("ACR", 0x0)
        self.write_register("WPMR", 0x0)
        self.write_register("WPSR", 0x0)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "OVER":
            # read to clear
            self.write_register(name, 0)
        elif name == "LCDR":
            logger.debug(f"[{name_:16s}]: Last ADC result, 0x{data & 0xFFF:08x}")
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            if data & (1 << ADC_CR.START):
                chsr = self.read_register("CHSR")
                if chsr != 0:
                    isr = chsr | (1 << ADC_ISR.DRDY)
                    self.write_register("ISR", isr)
                    enabled = []
                    for i in range(16):
                        if chsr & 0x01:
                            enabled.append(i)
                        chsr >>= 1
                    mr = self.read_register("MR")
                    if mr & (1 << ADC_MR.USEQ):
                        seq = []
                        seqr1 = self.read_register("SEQR1")
                        seqr2 = self.read_register("SEQR2")
                        for seqr in [seqr1, seqr2]:
                            for i in range(8):
                                ch = seqr & 0x0F
                                if ch in enabled:
                                    seq.append(ch)
                                seqr >>= 4
                        last_ch = seq[-1]
                    else:
                        last_ch = enabled[-1]
                    cdr = self.read_register(f"CDR{last_ch}")
                    emr = self.read_register("EMR")
                    lcdr = cdr
                    if emr & (1 << ADC_EMR.TAG):
                        lcdr |= last_ch << 12
                    self.write_register("LCDR", lcdr)
        elif name in ["CHER"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                diff = val ^ new_val
                for i in range(32):
                    if diff & 0x01:
                        logger.debug(f"[{name_:16s}]: Enable {self.NAME}{i}")
                    diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["CHDR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                diff = val ^ new_val
                for i in range(32):
                    if diff & 0x01:
                        logger.debug(f"[{name_:16s}]: Disable {self.NAME}{i}")
                    diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["MR", "SEQR1", "SEQR2", "EMR", "CWR", "CGR", "COR", "ACR"]:
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x41444300:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data


class UART_CR(IntEnum):
    RSTRX = 2
    RSTTX = 3
    RXEN = 4
    RXDIS = 5
    TXEN = 6
    TXDIS = 7
    RSTSTA = 8


class UART_SR(IntEnum):
    RXRDY = 0
    TXRDY = 1
    ENDRX = 3
    ENDTX = 4
    OVRE = 5
    FRAME = 6
    PARE = 7
    TXEMPTY = 9
    TXBUFE = 11
    RXBUFF = 12


class ArmSamUart(ArmHardwareBase):
    """Universal Asynchronous Receiver Transceiver"""

    NAME = "UART"
    REGISTERS = (
        ("CR", "I", 0xFFFFFFFF),
        ("MR", "I", 0xFFFFFFFF),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("SR", "I", 0x00000000),
        ("RHR", "I", 0x00000000),
        ("THR", "I", 0xFFFFFFFF),
        ("BRGR", "I", 0xFFFFFFFF),
    )
    COMMAND = "socat -d -d pty,link={uart},raw,echo=0 pty,link={tty},raw,echo=0"

    def __init__(self, *args, **kwargs: Any) -> None:
        from tempfile import mkstemp
        import shlex
        from subprocess import Popen, PIPE, TimeoutExpired

        super().__init__(*args, **kwargs)
        self._last_rx = 0
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # ref: https://stackoverflow.com/questions/2291772/virtual-serial-device-in-python
        uart_ = mkstemp(prefix=b"uart_")
        tty_ = mkstemp(prefix=b"tty_")
        os.close(uart_[0])
        os.close(tty_[0])
        self._uart = uart_[1].decode("utf-8")
        self._tty = tty_[1].decode("utf-8")
        self._proc = Popen(shlex.split(self.COMMAND.format(uart=self._uart, tty=self._tty)), stdout=PIPE, stderr=PIPE)
        try:
            _ = self._proc.communicate(timeout=1)
        except TimeoutExpired:
            pass
        self._serial = Serial(self._uart, kwargs.get("baudrate", 115200))
        logger.info(f"Serial device: {self._tty}")

    def __del__(self):
        self._proc.kill()
        os.remove(self._uart)
        os.remove(self._tty)

    def reset(self):
        super().reset()
        self.write_register("MR", 0x00000000)
        self.write_register("IMR", 0x00000000)
        self.write_register("SR", 0x00000000)
        self.write_register("RHR", 0x00000000)
        self.write_register("BRGR", 0x00000000)

    @property
    def tty_device(self):
        return self._tty

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "SR":
            sr = self.read_register("SR")
            if self._serial.in_waiting > 0:
                sr |= 1 << UART_SR.RXRDY
            # if self._serial.in_waiting > 1:
            #     sr |= 1 << UART_SR.OVRE
            # if self._serial.out_waiting <= 1:
            #     sr |= 1 << UART_SR.TXRDY
            sr |= 1 << UART_SR.TXRDY
            self.write_register("SR", sr)
        elif name == "RHR":
            if self._serial.in_waiting > 0:
                # self._last_rx = int.from_bytes(self._serial.read(self._serial.in_waiting + 10)[-1], "little")
                self._last_rx = int.from_bytes(self._serial.read(1), "little")
            data = self._last_rx
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            sr = self.read_register("SR")
            sr_orig = sr
            if data & (1 << UART_CR.RXEN):
                sr |= 1 << UART_SR.RXRDY
            if data & (1 << UART_CR.TXEN):
                sr |= 1 << UART_SR.TXRDY
            if data & ((1 << UART_CR.RSTRX) | (1 << UART_CR.RXDIS)):
                sr &= ~(1 << UART_SR.RXRDY)
                if data & (1 << UART_CR.RSTRX):
                    self._serial.reset_input_buffer()
            if data & ((1 << UART_CR.RSTTX) | (1 << UART_CR.TXDIS)):
                sr &= ~(1 << UART_SR.TXRDY)
                if data & (1 << UART_CR.RSTTX):
                    self._serial.reset_output_buffer()
            if data & (1 << UART_CR.RSTSTA):
                sr &= ~((1 << UART_SR.OVRE) | (1 << UART_SR.FRAME) | (1 << UART_SR.PARE))
            self.write_register("SR", sr)
            diff = sr_orig ^ sr
            if diff & (1 << UART_SR.RXRDY):
                if sr & (1 << UART_SR.RXRDY):
                    logger.debug(f"[{name_:16s}]: Enable RX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable RX")
            if diff & (1 << UART_SR.TXRDY):
                if sr & (1 << UART_SR.TXRDY):
                    logger.debug(f"[{name_:16s}]: Enable TX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable TX")
        elif name in ["IER"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            data = 0
        elif name in ["IDR"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val & ~data
            self.write_register(reg, new_val)
        if name == "THR":
            # if self._serial.out_waiting <= 1:
            #     self._serial.write(data.to_bytes(1, "little"))
            self._serial.write(data.to_bytes(1, "little"))
            # logger.debug(f'[{name_:16s}]: Output "{data.to_bytes(1, "little")}"')
        return data


class ArmSamPwm(ArmHardwareBase):
    """Pulse Width Modulation"""
    # TODO: CLK selection?

    NAME = "PWM"
    REGISTERS = (
        ("CLK", "I", 0xFFFFFFFF),
        ("ENA", "I", 0xFFFFFFFF),
        ("DIS", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("IER1", "I", 0xFFFFFFFF),
        ("IDR1", "I", 0xFFFFFFFF),
        ("IMR1", "I", 0x00000000),
        ("ISR1", "I", 0x00000000),
        ("SCM", "I", 0xFFFFFFFF),
        ("RESERVED0", "I", 0x00000000),
        ("SCUC", "I", 0xFFFFFFFF),
        ("SCUP", "I", 0xFFFFFFFF),
        ("SCUPUPD", "I", 0xFFFFFFFF),
        ("IER2", "I", 0xFFFFFFFF),
        ("IDR2", "I", 0xFFFFFFFF),
        ("IMR2", "I", 0x00000000),
        ("ISR2", "I", 0x00000000),
        ("OOV", "I", 0xFFFFFFFF),
        ("OS", "I", 0xFFFFFFFF),
        ("OSS", "I", 0xFFFFFFFF),
        ("OSC", "I", 0xFFFFFFFF),
        ("OSSUPD", "I", 0xFFFFFFFF),
        ("OSCUPD", "I", 0xFFFFFFFF),
        ("FMR", "I", 0xFFFFFFFF),
        ("FSR", "I", 0x00000000),
        ("FCR", "I", 0xFFFFFFFF),
        ("FPV", "I", 0xFFFFFFFF),
        ("FPE1", "I", 0xFFFFFFFF),
        ("FPE2", "I", 0xFFFFFFFF),
        ("RESERVED1", "2I", 0x00000000),
        ("ELMR0", "I", 0xFFFFFFFF),
        ("ELMR1", "I", 0xFFFFFFFF),
        ("RESERVED2", "11I", 0x00000000),
        ("SMMR", "I", 0xFFFFFFFF),
        ("RESERVED3", "12I", 0x00000000),
        ("WPCR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
        ("RESERVED4", "17I", 0x00000000),
        ("CMPV0", "I", 0xFFFFFFFF),
        ("CMPVUPD0", "I", 0xFFFFFFFF),
        ("CMPM0", "I", 0xFFFFFFFF),
        ("CMPMUPD0", "I", 0xFFFFFFFF),
        ("CMPV1", "I", 0xFFFFFFFF),
        ("CMPVUPD1", "I", 0xFFFFFFFF),
        ("CMPM1", "I", 0xFFFFFFFF),
        ("CMPMUPD1", "I", 0xFFFFFFFF),
        ("CMPV2", "I", 0xFFFFFFFF),
        ("CMPVUPD2", "I", 0xFFFFFFFF),
        ("CMPM2", "I", 0xFFFFFFFF),
        ("CMPMUPD2", "I", 0xFFFFFFFF),
        ("CMPV3", "I", 0xFFFFFFFF),
        ("CMPVUPD3", "I", 0xFFFFFFFF),
        ("CMPM3", "I", 0xFFFFFFFF),
        ("CMPMUPD3", "I", 0xFFFFFFFF),
        ("CMPV4", "I", 0xFFFFFFFF),
        ("CMPVUPD4", "I", 0xFFFFFFFF),
        ("CMPM4", "I", 0xFFFFFFFF),
        ("CMPMUPD4", "I", 0xFFFFFFFF),
        ("CMPV5", "I", 0xFFFFFFFF),
        ("CMPVUPD5", "I", 0xFFFFFFFF),
        ("CMPM5", "I", 0xFFFFFFFF),
        ("CMPMUPD5", "I", 0xFFFFFFFF),
        ("CMPV6", "I", 0xFFFFFFFF),
        ("CMPVUPD6", "I", 0xFFFFFFFF),
        ("CMPM6", "I", 0xFFFFFFFF),
        ("CMPMUPD6", "I", 0xFFFFFFFF),
        ("CMPV7", "I", 0xFFFFFFFF),
        ("CMPVUPD7", "I", 0xFFFFFFFF),
        ("CMPM7", "I", 0xFFFFFFFF),
        ("CMPMUPD7", "I", 0xFFFFFFFF),
        ("RESERVED5", "20I", 0x00000000),
        ("CMR0", "I", 0xFFFFFFFF),
        ("CDTY0", "I", 0xFFFFFFFF),
        ("CDTYUPD0", "I", 0xFFFFFFFF),
        ("CPRD0", "I", 0x0000FFFF),
        ("CPRDUPD0", "I", 0x0000FFFF),
        ("CCNT0", "I", 0x00000000),
        ("DT0", "I", 0xFFFFFFFF),
        ("DTUPD0", "I", 0xFFFFFFFF),
        ("CMR1", "I", 0xFFFFFFFF),
        ("CDTY1", "I", 0xFFFFFFFF),
        ("CDTYUPD1", "I", 0xFFFFFFFF),
        ("CPRD1", "I", 0x0000FFFF),
        ("CPRDUPD1", "I", 0x0000FFFF),
        ("CCNT1", "I", 0x00000000),
        ("DT1", "I", 0xFFFFFFFF),
        ("DTUPD1", "I", 0xFFFFFFFF),
        ("CMR2", "I", 0xFFFFFFFF),
        ("CDTY2", "I", 0xFFFFFFFF),
        ("CDTYUPD2", "I", 0xFFFFFFFF),
        ("CPRD2", "I", 0x0000FFFF),
        ("CPRDUPD2", "I", 0x0000FFFF),
        ("CCNT2", "I", 0x00000000),
        ("DT2", "I", 0xFFFFFFFF),
        ("DTUPD2", "I", 0xFFFFFFFF),
        ("CMR3", "I", 0xFFFFFFFF),
        ("CDTY3", "I", 0xFFFFFFFF),
        ("CDTYUPD3", "I", 0xFFFFFFFF),
        ("CPRD3", "I", 0x0000FFFF),
        ("CPRDUPD3", "I", 0x0000FFFF),
        ("CCNT3", "I", 0x00000000),
        ("DT3", "I", 0xFFFFFFFF),
        ("DTUPD3", "I", 0xFFFFFFFF),
        ("CMR4", "I", 0xFFFFFFFF),
        ("CDTY4", "I", 0xFFFFFFFF),
        ("CDTYUPD4", "I", 0xFFFFFFFF),
        ("CPRD4", "I", 0x0000FFFF),
        ("CPRDUPD4", "I", 0x0000FFFF),
        ("CCNT4", "I", 0x00000000),
        ("DT4", "I", 0xFFFFFFFF),
        ("DTUPD4", "I", 0xFFFFFFFF),
        ("CMR5", "I", 0xFFFFFFFF),
        ("CDTY5", "I", 0xFFFFFFFF),
        ("CDTYUPD5", "I", 0xFFFFFFFF),
        ("CPRD5", "I", 0x0000FFFF),
        ("CPRDUPD5", "I", 0x0000FFFF),
        ("CCNT5", "I", 0x00000000),
        ("DT5", "I", 0xFFFFFFFF),
        ("DTUPD5", "I", 0xFFFFFFFF),
        ("CMR6", "I", 0xFFFFFFFF),
        ("CDTY6", "I", 0xFFFFFFFF),
        ("CDTYUPD6", "I", 0xFFFFFFFF),
        ("CPRD6", "I", 0x0000FFFF),
        ("CPRDUPD6", "I", 0x0000FFFF),
        ("CCNT6", "I", 0x00000000),
        ("DT6", "I", 0xFFFFFFFF),
        ("DTUPD6", "I", 0xFFFFFFFF),
        ("CMR7", "I", 0xFFFFFFFF),
        ("CDTY7", "I", 0xFFFFFFFF),
        ("CDTYUPD7", "I", 0xFFFFFFFF),
        ("CPRD7", "I", 0x0000FFFF),
        ("CPRDUPD7", "I", 0x0000FFFF),
        ("CCNT7", "I", 0x00000000),
        ("DT7", "I", 0xFFFFFFFF),
        ("DTUPD7", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._step = kwargs.get("step", 1)
        self._ccnt = [0] * 8
        self._cprd = [0] * 8
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CLK", 0x0)
        self.write_register("SR", 0x0)
        self.write_register("IMR1", 0x0)
        self.write_register("ISR1", 0x0)
        self.write_register("SCM", 0x0)
        self.write_register("SCUC", 0x0)
        self.write_register("SCUP", 0x0)
        self.write_register("SCUPUPD", 0x0)
        self.write_register("IMR2", 0x0)
        self.write_register("ISR2", 0x0)
        self.write_register("OOV", 0x0)
        self.write_register("OS", 0x0)
        self.write_register("FMR", 0x0)
        self.write_register("FSR", 0x0)
        self.write_register("FPV", 0x0)
        self.write_register("FPE1", 0x0)
        self.write_register("FPE2", 0x0)
        self.write_register("ELMR0", 0x0)
        self.write_register("ELMR1", 0x0)
        self.write_register("SMMR", 0x0)
        self.write_register("WPSR", 0x0)
        self.write_register("CMPV0", 0x0)
        self.write_register("CMPM0", 0x0)
        self.write_register("CMPV1", 0x0)
        self.write_register("CMPM1", 0x0)
        self.write_register("CMPV2", 0x0)
        self.write_register("CMPM2", 0x0)
        self.write_register("CMPV3", 0x0)
        self.write_register("CMPM3", 0x0)
        self.write_register("CMPV4", 0x0)
        self.write_register("CMPM4", 0x0)
        self.write_register("CMPV5", 0x0)
        self.write_register("CMPM5", 0x0)
        self.write_register("CMPV6", 0x0)
        self.write_register("CMPM6", 0x0)
        self.write_register("CMPV7", 0x0)
        self.write_register("CMPM7", 0x0)
        # self.write_register("CMR", 0x0)
        # self.write_register("CDTY", 0x0)
        # self.write_register("CPRD", 0x0)
        # self.write_register("CCNT", 0x0)
        # self.write_register("DT", 0x0)
        self._ccnt = [0] * 8
        self._cprd = [0] * 8

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        for i in range(8):
            self._ccnt[i] += self._step
            if self._ccnt[i] >= self._cprd[i]:
                self._ccnt[i] = 0
                # TODO: trigger interrupt
                # csr |= 1 << CSR.COUNTFLAG
                # if csr & (1 << CSR.TICKINT):
                #     self._ctl.set_irq_pending(self._irq)
                # self.write_register("CSR", csr)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "WPSR":
            data_ = data & 0x0000FF7F
            # read to clear
            self.write_register(name, data_)
        # elif name == "LCDR":
        #     logger.debug(f"[{name_:16s}]: Last ADC result, 0x{data & 0xFFF:08x}")
        elif name[:-1] == "CCNT":
            data = self._ccnt[int(name[-1])]
            self.write_register(f"CCNT{name[-1]}", self._ccnt[int(name[-1])])
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["ENA", "IER1", "IER2"]:
            if name == "ENA":
                reg = "SR"
            elif name[-1] in "1234567890":
                reg = name[:-3] + "M" + name[-2:]
            else:
                reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            if name == "ENA":
                diff = val ^ new_val
                # 8 channels
                for i in range(8):
                    if diff & 0x01:
                        # clear counter
                        self.write_register(f"CCNT{i}", 0)
                        logger.debug(f"[{name_:16s}]: Enable {self.NAME}{i}")
                    diff >>= 1
            data = 0
        elif name in ["DIS", "IDR1", "IDR2"]:
            if name == "DIS":
                wpsr = self.read_register("WPSR")
                wps1 = (wpsr | (wpsr >> 8)) & 0x02
                if wps1:
                    logger.warning(f"[{name_:16s}]: Ignore write")
                    reg = None
                else:
                    reg = "SR"
            elif name[-1] in "1234567890":
                reg = name[:-3] + "M" + name[-2:]
            else:
                reg = name[:-2] + "M" + name[-1:]
            if reg:
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                if name == "DIS":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable {self.NAME}{i}")
                        diff >>= 1
            data = 0
        elif name == "CLK":
            wpsr = self.read_register("WPSR")
            wps0 = (wpsr | (wpsr >> 8)) & 0x01
            if wps0:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name in ["SCM", "SMMR"] or name[:-1] in ["CMR"]:
            wpsr = self.read_register("WPSR")
            wps2 = (wpsr | (wpsr >> 8)) & 0x04
            if wps2:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name[:-1] in ["CPRD", "CPRDUPD"]:
            wpsr = self.read_register("WPSR")
            wps3 = (wpsr | (wpsr >> 8)) & 0x08
            if wps3:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
            else:
                self._cprd[int(name[-1])] = data
                if name[:-1] == "CPRDUPD":
                    self.write_register(f"CPRD{name[-1]}", data)
                    data = 0
        elif name[:-1] in ["DT", "DTUPD"]:
            wpsr = self.read_register("WPSR")
            wps4 = (wpsr | (wpsr >> 8)) & 0x10
            if wps4:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
            elif name[:-1] == "DTUPD":
                self.write_register(f"DT{name[-1]}", data)
                data = 0
        elif name in ["FMR", "FPV", "FPE1", "FPE2"]:
            wpsr = self.read_register("WPSR")
            wps5 = (wpsr | (wpsr >> 8)) & 0x20
            if wps5:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name == "SCUPUPD":
            scup = self.read_register("SCUP")
            scup = (scup & 0xFFFFFFF0) | (data & 0x0000000F)
            self.write_register("SCUP", scup)
            data = 0
        elif name == "WPCR":
            if (data & 0xFFFFFF00) != 0x50574D00:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0xFF
                wpcmd = data & 0x03
                wprg = (data >> 2) & 0x3F
                wpsr = self.read_register("WPSR")
                if wpcmd == 0:
                    wpsr &= ~wprg
                elif wpcmd == 1:
                    wpsr |= wprg
                elif wpcmd == 2:
                    wpsr |= (wprg << 8)
                self.write_register("WPSR", wpsr)
        return data


class UOTGHS_CTRL(IntEnum):
    FRZCLK = 14
    USBE = 15


class UOTGHS_SR(IntEnum):
    CLKUSABLE = 14


class ArmSamUotghs(ArmHardwareBase):
    """USB On-The-Go interface"""

    NAME = "UOTGHS"
    REGISTERS = (
        ("CTRL", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("SCR", "I", 0xFFFFFFFF),
        ("SFR", "I", 0xFFFFFFFF),
        ("RESERVED0", "8I", 0x00000000),
        ("FSM", "I", 0x00000000),

    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CTRL", 0x03004000)
        self.write_register("SR", 0x00000400)
        self.write_register("FSM", 0x00000009)

    # def fix_after_read(self, name: str, register: Register, data: int) -> int:
    #     if name == "SR":

    #     return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "CTRL":
            if data & (1 << UOTGHS_CTRL.FRZCLK):
                sr = self.read_register("SR")
                sr |= (1 << UOTGHS_SR.CLKUSABLE)
                self.write_register("SR", sr)
        return data


class ArmStmRcc(ArmHardwareBase):
    """Reset and clock control"""

    NAME = "RCC"
    REGISTERS = (
        ("CR", "I", 0x050D00F9),
        ("PLLCFGR", "I", 0x0F437FFF),
        ("CFGR", "I", 0xFFFFFCF3),
        ("CIR", "I", 0x00BF3F00),
        ("AHB1RSTR", "I", 0x0060109F),
        ("AHB2RSTR", "I", 0x00000080),
        ("RESERVED0", "2I", 0x00000000),
        ("APB1RSTR", "I", 0x10E2C80F),
        ("APB2RSTR", "I", 0x00177931),
        ("RESERVED1", "2I", 0x00000000),
        ("AHB1ENR", "I", 0x0060109F),
        ("AHB2ENR", "I", 0x00000080),
        ("RESERVED2", "2I", 0x00000000),
        ("APB1ENR", "I", 0x10E2C80F),
        ("APB2ENR", "I", 0x00177931),
        ("RESERVED3", "2I", 0x00000000),
        ("AHB1LPENR", "I", 0x0061909F),
        ("AHB2LPENR", "I", 0x00000080),
        ("RESERVED4", "2I", 0x00000000),
        ("APB1LPENR", "I", 0x10E2C80F),
        ("APB2LPENR", "I", 0x00177931),
        ("RESERVED5", "2I", 0x00000000),
        ("BDCR", "I", 0x0001830D),
        ("CSR", "I", 0x01000001),
        ("RESERVED6", "2I", 0x00000000),
        ("SSCGR", "I", 0xCFFFFFFF),
        ("PLLI2SCFGR", "I", 0x70007FFF),
        ("RESERVED7", "I", 0x00000000),
        ("DCKCFGR", "I", 0x01000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CR", 0x00000081)
        self.write_register("PLLCFGR", 0x24003010)
        self.write_register("CFGR", 0x0)
        self.write_register("CIR", 0x0)
        self.write_register("AHB1RSTR", 0x0)
        self.write_register("AHB2RSTR", 0x0)
        self.write_register("APB1RSTR", 0x0)
        self.write_register("APB2RSTR", 0x0)
        self.write_register("AHB1ENR", 0x0)
        self.write_register("AHB2ENR", 0x0)
        self.write_register("APB1ENR", 0x0)
        self.write_register("APB2ENR", 0x0)
        self.write_register("AHB1LPENR", 0x0061900F)
        self.write_register("AHB2LPENR", 0x00000080)
        self.write_register("APB1LPENR", 0x10E2C80F)
        self.write_register("APB2LPENR", 0x00077930)
        self.write_register("BDCR", 0x00077930)
        self.write_register("CSR", 0x0E000000)
        self.write_register("SSCGR", 0x0)
        self.write_register("PLLI2SCFGR", 0x24003000)
        self.write_register("DCKCFGR", 0x0)

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            # set ready, if enabled clock
            clock_mask = 0x05010001
            status_mask = clock_mask << 1
            clock = data & clock_mask
            logger.debug(f"[{name_:16s}]: Enable clock 0x{clock:08x}")
            data = (data & ~status_mask) | (clock << 1)
        elif name == "CFGR":
            # set status, if selected clock
            clock_mask = 0x00000003
            status_mask = clock_mask << 2
            clock = data & clock_mask
            # clock = data & 0x00000003
            logger.debug(f"[{name_:16s}]: Select clock 0x{clock:08x}")
            data = (data & ~status_mask) | (clock << 2)
        return data


class ArmStmGpio(ArmHardwareBase):
    """General-purpose I/Os"""

    NAME = "GPIO"
    REGISTERS = (
        ("MODER", "I", 0xFFFFFFFF),
        ("OTYPER", "I", 0x0000FFFF),
        ("OSPEEDR", "I", 0xFFFFFFFF),
        ("PUPDR", "I", 0xFFFFFFFF),
        ("IDR", "I", 0x00000000),  # word access only
        ("ODR", "I", 0x0000FFFF),
        ("BSRR", "I", 0xFFFFFFFF),
        ("LCKR", "I", 0x0001FFFF),
        ("AFRL", "I", 0xFFFFFFFF),
        ("AFRH", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.NAME = self._name.upper()
        self._fix_before_write = self.fix_before_write
        self._lock_seq = 0
        self._lock_val = 0

    def reset(self):
        super().reset()
        if self.NAME.endswith("A"):
            self.write_register("MODER", 0xA8000000)
            self.write_register("OSPEEDR", 0x0C000000)
            self.write_register("PUPDR", 0x64000000)
        if self.NAME.endswith("B"):
            self.write_register("MODER", 0x00000280)
            self.write_register("OSPEEDR", 0x000000C0)
            self.write_register("PUPDR", 0x00000100)
        else:
            self.write_register("MODER", 0x0)
            self.write_register("OSPEEDR", 0x0)
            self.write_register("PUPDR", 0x0)
        self.write_register("OTYPER", 0x0)
        self.write_register("IDR", 0x0)
        self.write_register("ODR", 0x0)
        self.write_register("BSRR", 0x0)
        self.write_register("LCKR", 0x0)
        self.write_register("AFRL", 0x0)
        self.write_register("AFRH", 0x0)

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if self._lock_seq >= 3:
            logger.debug(f"[{name_:16s}]: Config is locked, 0x{self._lock_val:08x}")
            if name in ["OTYPER"]:
                mask = self._lock_val & 0xFFFF
                data = (data & ~mask) | (data_orig & mask)
            elif name in ["MODER", "OSPEEDR", "PUPDR"]:
                val = self._lock_val
                mask = 0
                for i in range(16):
                    val_ = val & 0x1
                    if val_:
                        mask |= 0x3
                    val >>= 1
                    mask <<= 2
                data = (data & ~mask) | (data_orig & mask)
            elif name in ["AFRL", "AFRH"]:
                val = self._lock_val
                mask = 0
                for i in range(16):
                    val_ = val & 0x1
                    if val_:
                        mask |= 0xF
                    val >>= 1
                    mask <<= 4
                if name == "AFRH":
                    mask >>= 32
                mask &= 0xFFFFFFFF
                data = (data & ~mask) | (data_orig & mask)
        if name == "ODR":
            odr = data_orig
            diff = odr ^ data
            for i in range(16):
                diff_ = diff & 0x01
                odr_ = odr & 0x01
                if diff_:
                    if odr_:
                        logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                    else:
                        logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                diff >>= 1
                odr >>= 1
        elif name == "BSRR":
            logger.debug(f"[{name_:16s}]: BSRR <= 0x{data:08x}")
            br = (data >> 16) & 0xFFFF
            bs = data & 0xFFFF
            odr = self.read_register("ODR")
            odr = ((odr & ~br) | bs) & 0xFFFF
            self.write_register("ODR", odr)
            data = 0x0
            for i in range(16):
                br_ = br & 0x01
                bs_ = bs & 0x01
                if bs_:
                    logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                elif br_:
                    logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                br >>= 1
                bs >>= 1
        elif name == "LCKR":
            if self._lock_seq <= 0 and (data & 0x10000) != 0x0:
                self._lock_seq = 1
                self._lock_val = data
                data = 0
            elif self._lock_seq in [1, 2]:
                if (data ^ self._lock_val) & 0x1FFFF == 0x10000:
                    self._lock_seq += 1
                    self._lock_val = data
                else:
                    self._lock_seq = 0
                    self._lock_val = 0
                    data = 0
            elif self._lock_seq >= 4:
                logger.warning(f"[{name_:16s}]: LCKR is already locked")
                data = data_orig
            logger.debug(f"[{name_:16s}]: LCKR write sequence: {self._lock_seq}")
            if self._lock_seq == 3:
                self._lock_seq += 1
        return data


HARDWARE_MAPPING["cortex_m"] = {
    "scid": ArmHardwareScid,
    "systick": ArmHardwareSystick,
    "nvic": ArmHardwareNvic,
    "scb": ArmHardwareScb,
    "cp": ArmHardwareCp,
    "dbg": ArmHardwareDbg,
    "dwt": ArmHardwareDwt,
    "pmc": ArmSamPmc,
    "rcc": ArmStmRcc,
    "gpio": {
        "stm": ArmStmGpio,
        "sam": ArmSamGpio,
    },
    "adc": ArmSamAdc,
    "pwm": ArmSamPwm,
    "uart": ArmSamUart,
    "uotghs": ArmSamUotghs,
}
