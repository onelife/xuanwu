# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
from collections import OrderedDict, namedtuple
from struct import Struct
from enum import IntEnum
from functools import partial
from typing import Optional, Union, Any, Dict, Tuple, Set

from unicorn import Uc, UC_HOOK_CODE, UC_HOOK_INTR
from unicorn.arm_const import *

from ..config import logger, EXCP
from ..register import RegisterController
from ..memory import MemoryController
from ..exception import XwUnknownHardware, XwInvalidMemoryAddress, XwInvalidMemorySize


__all__ = ["arm_core_registers", "arm_context_registers", "Register", "ArmHardwareController", "ArmHardwareBase"]

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

Register = namedtuple("Register", ["format", "offset", "mask"])
IrqOp = namedtuple("IrqOp", ["is_enabled", "set_pending", "set_active", "get_priority"])


class Exception_(IntEnum):
    ThreadMode = 0
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
    nPRIV = 0
    SPSEL = 1
    FPCA = 2


class EPSR(IntEnum):
    T = 24


class ICSR(IntEnum):
    PENDSTCLR = 25
    PENDSTSET = 26
    PENDSVCLR = 27
    PENDSVSET = 28
    NMIPENDSET = 31


class CCR(IntEnum):
    NONBASETHRDENA = 0
    STKALIGN = 9


class CFSR(IntEnum):
    # MemManage = 0
    # BusFault = 8
    # UsageFault = 16
    INVPC = 18


class ArmHardwareController(object):
    """Armv7-m hardware control unit"""

    def __init__(self, box: Uc, reg: RegisterController, mem: MemoryController, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._box = box
        self._reg = reg
        self._mem = mem
        self._thread_mode = True
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

    def is_irq_pending_or_active(self, irq: int) -> bool:
        return irq in self._irq_pending or irq in self._irq_handling

    def set_irq_pending(self, irq: int) -> None:
        if irq not in self._irq_pending:
            # logger.debug(f"Interrupt: Add IRQ_{irq} in pending queue")
            self._irq_pending.append(irq)
            self._irq_op[irq].set_pending(state=True)

    def clear_irq_pending(self, irq: int) -> None:
        if irq in self._irq_pending:
            logger.debug(f"Interrupt: Remove IRQ{irq} from pending queue")
            self._irq_pending.remove(irq)
            self._irq_op[irq].set_pending(state=False)

    def push_context(self, exception: int) -> None:
        control = self._reg.read("control")
        # if HaveFPExt() && CONTROL.FPCA == '1' then
        # if control & (1 << CONTROL.FPCA) and False:
        #     frame_size = 0x68
        #     force_align = True
        # else:
        frame_size = 0x20
        ccr = self.perif["scb"].read_register("CCR")
        force_align = ccr & (1 << CCR.STKALIGN)
        if force_align:
            sp_mask = ~0x4
        else:
            sp_mask = ~0x0
        if control & (1 << CONTROL.SPSEL) and self._thread_mode:
            sp = self._reg.read("psp")
            frame_ptr_align = force_align and (sp & 0x4)
            sp = (sp & sp_mask) - frame_size
            self._reg.write("psp", sp)
            # logger.debug(f"push_context: p{sp = :08x}")
        else:
            sp = self._reg.read("msp")
            frame_ptr_align = force_align and (sp & 0x4)
            sp = (sp & sp_mask) - frame_size
            self._reg.write("msp", sp)
            # logger.debug(f"push_context: m{sp = :08x}")

        for i in range(8):
            reg = arm_context_registers[i]
            val = self._reg.read(reg)
            if reg == "pc" and exception in [Exception_.MemManage, Exception_.UsageFault]:
                val -= 4
            elif reg == "xpsr":
                val = (val & 0xFFFFFDFF) | (frame_ptr_align << 9)
                # logger.debug(f"push_context: {reg} = {val:08x}")
            self._mem.write(sp, self.format_.pack(val))
            sp += 4

        # TODO: if HaveFPExt() && CONTROL.FPCA == '1' then

        if self._thread_mode:
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
        # logger.debug(f"EXP_{exception} {exc_return = :08x}")
        self._reg.write("lr", exc_return)

    def pop_context(self, sp: int, exc_return: int) -> None:
        # TODO: if HaveFPExt() && EXC_RETURN<4> == '0' then
        frame_size = 0x20
        ccr = self.perif["scb"].read_register("CCR")
        force_align = ccr & (1 << CCR.STKALIGN)

        psr = 0
        for i in range(8):
            reg = arm_context_registers[i]
            data = self._mem.read(sp, 4)
            val = self.format_.unpack(data)[0]
            if reg == "xpsr":
                psr = val
                # logger.debug(f"pop_context: {reg} = {val:08x}")
            else:
                self._reg.write(reg, val)
            sp += 4

        # TODO: if HaveFPExt() then
        if force_align and (psr & 0x200):
            sp_mask = 0x4
        else:
            sp_mask = 0x0

        config = exc_return & 0xF
        if config in [0x1, 0x9]:
            sp = self._reg.read("msp")
            sp = (sp + frame_size) | sp_mask
            self._reg.write("msp", sp)
            logger.info(f"pop_context: m{sp = :08x}")
        elif config == 0xD:
            sp = self._reg.read("psp")
            sp = (sp + frame_size) | sp_mask
            self._reg.write("psp", sp)
            # logger.debug(f"pop_context: p{sp = :08x}")

        apsr = self._reg.read("apsr")
        apsr = (apsr & 0x07FFFFFF) | (psr & 0xF8000000)
        self._reg.write("apsr", apsr)

        ipsr = self._reg.read("ipsr")
        ipsr = (ipsr & 0xFFFFFE00) | (psr & 0x000001FF)
        self._reg.write("ipsr", ipsr)
        # logger.debug(f"pop_context: {ipsr = :08x}")

        epsr = self._reg.read("epsr")
        epsr = (epsr & 0xF8FF03FF) | (psr & 0x0700FC00)
        self._reg.write("epsr", epsr)

    def jump_isr(self, irq: int) -> None:
        # get handler address
        vector = self.perif["scb"].read_register("VTOR") & 0xFFFFFF80
        exp = irq + 16
        offset = vector | (exp << 2)
        addr = self.format_.unpack(self._mem.read(offset, 4))[0]
        tbit = addr & 0x1
        addr &= ~0x1
        # logger.debug(f"IRQ_{irq} handler @0x{addr:08x}")
        self._thread_mode = False
        if tbit:
            epsr = 1 << EPSR.T
        else:
            epsr = 0
        control = self._reg.read("control")
        control &= ~((1 << CONTROL.SPSEL) | (1 << CONTROL.FPCA))
        # set registers
        self._reg.write("pc", addr)
        self._reg.write("ipsr", exp)
        self._reg.write("epsr", epsr)
        self._reg.write("control", control)

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

    def trigger_usage_fault(self, exc_return: int):
        # update registers
        cfsr = self.perif["scb"].read_register("CFSR")
        cfsr |= 1 << CFSR.INVPC
        self.perif["scb"].write_register("CFSR", cfsr)
        self._reg.write("lr", exc_return | 0xF0000000)
        # UsageFault
        logger.warning(f"UsageFault: {exc_return = :08x}")
        self.jump_isr(Exception_.UsageFault - 16)

    def system_interrupt_callback(self, box: Uc, intno: int, data: Any):
        if intno == EXCP.EXCEPTION_EXIT:
            if self._thread_mode:
                logger.error(f"Return from ISR but in thread mode!?")
                raise RuntimeError(f"Return from ISR but in thread mode!?")
            pc = self._reg.pc_t
            # logger.debug(f"pc = {pc:08x}")
            # TODO: if HaveFPExt() then
            if (pc & 0x0FFFFFF0) != 0x0FFFFFF0:
                raise RuntimeError("UNPREDICTABLE")
            exp = self._reg.read("ipsr")
            # logger.debug(f"system_interrupt_callback: {exp =}, {self._irq_handling =}, {self._irq_pending =}")
            if exp == 0:
                logger.error(f"ipsr == {exp} when handling EXCEPTION_EXIT!?")
                raise RuntimeError(f"ipsr == {exp} when handling EXCEPTION_EXIT!?")
            irq = exp - 16
            # logger.debug(f"Exit IRQ_{irq} handler...")
            nested_activation = len(self._irq_handling)

            # deactivate
            self._irq_op[irq].set_active(state=False)
            if exp != Exception_.NMI:
                self._reg.write("faultmask", 0x0)

            if irq not in self._irq_handling:
                logger.error(f"Completed IRQ_{irq} not in handling list!?")
                return self.trigger_usage_fault(pc)

            self._irq_handling.remove(irq)
            _, next_irq, _ = self.get_next_irq()
            if next_irq:
                # tail-chaining
                self._irq_handling.append(next_irq)
                self._irq_pending.remove(next_irq)
                self._irq_op[next_irq].set_pending(state=False)
                # set active
                self._irq_op[next_irq].set_active(state=True)
                # jump to ISR
                logger.debug(f"Tail-chaining IRQ_{next_irq} handler...")
                self._reg.write("lr", pc | 0xF0000000)
                return self.jump_isr(next_irq)

            config = pc & 0xF
            if config == 0x1:
                sp = self._reg.read("msp")
                self._thread_mode = False
                control = self._reg.read("control")
                control &= ~(1 << CONTROL.SPSEL)
                self._reg.write("control", control)
            elif config == 0x9:
                ccr = self.perif["scb"].read_register("CCR")
                if nested_activation != 1 and (ccr & (1 << CCR.NONBASETHRDENA)) == 0:
                    return self.trigger_usage_fault(pc)
                else:
                    sp = self._reg.read("msp")
                    self._thread_mode = True
                    control = self._reg.read("control")
                    control &= ~(1 << CONTROL.SPSEL)
                    self._reg.write("control", control)
            elif config == 0xD:
                ccr = self.perif["scb"].read_register("CCR")
                if nested_activation != 1 and (ccr & (1 << CCR.NONBASETHRDENA)) == 0:
                    return self.trigger_usage_fault(pc)
                else:
                    sp = self._reg.read("psp")
                    self._thread_mode = True
                    control = self._reg.read("control")
                    control |= 1 << CONTROL.SPSEL
                    self._reg.write("control", control)
            else:
                return self.trigger_usage_fault(pc)

            # restore context
            if exp != Exception_.SysTick:
                logger.debug(f"Restore context after IRQ_{irq}")
            self.pop_context(sp, pc)

            if not self._thread_mode and exp == 0:
                self.push_context(Exception_.UsageFault)
                return self.trigger_usage_fault(pc)

        else:
            pc = self._reg.pc_t
            ipsr = self._reg.read("ipsr")
            logger.info(f"intno: {intno}, data: {data}, pc: {pc:08x}, ipsr: {ipsr:08x}")
            raise

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        if not self._irq_pending:
            return
        # logger.debug(f"{self._irq_pending =}, {self._irq_handling =}")
        is_preempted, next_irq, _ = self.get_next_irq()
        if not is_preempted:
            return
        # preempt
        exp = self._reg.read("ipsr")
        if exp == 0:
            if self._irq_handling:
                logger.error(f"IRQ handling list is not empty in thread mode!? {self._irq_handling}")
                raise RuntimeError(f"IRQ handling list is not empty in thread mode! {self._irq_handling}")
        # save context
        if next_irq + 16 != Exception_.SysTick:
            logger.debug(f"Save context before IRQ_{next_irq}")
        self.push_context(exp)
        self._irq_handling.append(next_irq)
        self._irq_pending.remove(next_irq)
        self._irq_op[next_irq].set_pending(state=False)
        # set active
        self._irq_op[next_irq].set_active(state=True)
        # jump to ISR
        # logger.debug(f"Enter IRQ_{next_irq} handler...")
        self.jump_isr(next_irq)

    def get_buildin(self, mode: str, name: str) -> Dict[str, Any]:
        buildin = {}
        # core
        if mode == "cortex_m":
            buildin = {
                "scid": ArmHardwareScid,
                "systick": ArmHardwareSystick,
                "nvic": ArmHardwareNvic,
                "scb": ArmHardwareScb,
                "cp": ArmHardwareCp,
                "dbg": ArmHardwareDbg,
                "dwt": ArmHardwareDwt,
            }
        else:
            raise XwUnknownHardware(f"Unknown Arm core: {mode}")
        # name
        if name == "sam":
            from .atmel_sam import BUILDIN
            buildin.update(BUILDIN)
        elif name == "stm":
            from .stm_stm import BUILDIN
            buildin.update(BUILDIN)
        else:
            raise XwUnknownHardware(f"Unknown Arm chip: {name}")
        return buildin

    def map_memory(self, chip: Dict[str, Any]) -> None:
        buildin = self.get_buildin(chip.get("mode"), chip["name"][:3].lower())
        if not buildin:
            return
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

    @staticmethod
    def _reg2irq(reg_val: int, reg_num: int) -> Set[int]:
        irqs = set()
        for i in range(32):
            if reg_val & 0x01:
                irqs.add(32 * reg_num + i)
            reg_val >>= 1
        return irqs

    @staticmethod
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
            logger.debug(f"[{name_:16s}]: Software trigger IRQ_{data}")
        return data


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
        # logger.debug(f"EXP_{exp}: pending {state}")
        if state:
            val |= 1 << offset
        else:
            val &= ~(1 << offset)
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
            data = data_orig
            if set_bits & (1 << ICSR.PENDSTCLR):
                self._ctl.clear_irq_pending(Exception_.SysTick - 16)
                data &= ~(1 << ICSR.PENDSTSET)
                set_bits &= ~(1 << ICSR.PENDSTSET)
            if set_bits & (1 << ICSR.PENDSVCLR):
                self._ctl.clear_irq_pending(Exception_.PendSV - 16)
                data &= ~(1 << ICSR.PENDSVSET)
                set_bits &= ~(1 << ICSR.PENDSVSET)
            if set_bits & (1 << ICSR.PENDSTSET):
                self._ctl.set_irq_pending(Exception_.SysTick - 16)
                data |= 1 << ICSR.PENDSTSET
            if set_bits & (1 << ICSR.PENDSVSET):
                self._ctl.set_irq_pending(Exception_.PendSV - 16)
                data |= 1 << ICSR.PENDSVSET
            if set_bits & (1 << ICSR.NMIPENDSET):
                self._ctl.set_irq_pending(Exception_.NMI - 16)
                data |= 1 << ICSR.NMIPENDSET
            # logger.warning(f"ICSR: {data_orig:08x} => {data:08x}")
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
