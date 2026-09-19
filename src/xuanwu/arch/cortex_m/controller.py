# -*- coding: utf-8 -*-

"""Armv7-M core: exception entry/exit, interrupt prioritisation and dispatch."""

from struct import Struct
from typing import Any, Dict, List, Optional, Tuple, Union

from unicorn import Uc, UC_HOOK_CODE, UC_HOOK_INTR

from ...config import EXCP, logger
from ...exception import XwUnknownHardware
from ...register import RegisterController
from ...memory import MemoryController
from ...backends.semihost import SEMIHOST_BKPT, SemiHosting
from ..base import IrqOp, arm_context_registers
from .constants import CCR, CFSR, CONTROL, EPSR, Exception_

__all__ = ["ArmHardwareController"]


class ArmHardwareController(object):
    """Armv7-m hardware control unit"""

    def __init__(
        self,
        box: Uc,
        reg: RegisterController,
        mem: MemoryController,
        options: Optional[Dict[str, Any]] = None,
        semihosting: Union[bool, SemiHosting, None] = True,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._box = box
        self._reg = reg
        self._mem = mem
        self._options = dict(options or {})
        # ``True`` builds a default service; pass an instance to capture its
        # output, or ``False``/``None`` to leave BKPT 0xAB unhandled.
        self._semihost = SemiHosting() if semihosting is True else (semihosting or None)
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
            logger.debug(f"pop_context: m{sp = :08x}")
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

    def is_semihost_call(self) -> bool:
        """True when the instruction at the PC is the semihosting trap ``BKPT 0xAB``."""
        address = self._reg.pc & ~0x1
        try:
            halfword = int.from_bytes(self._mem.read(address, 2), "little")
        except Exception as err:  # noqa: BLE001 - an unmapped PC means "not a trap"
            logger.debug(f"Cannot read the instruction at 0x{address:08x}: {err}")
            return False
        return halfword == SEMIHOST_BKPT

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

        elif intno == EXCP.BKPT and self._semihost is not None and self.is_semihost_call():
            # Unicorn implements QEMU's ARM semihosting but does not expose the
            # switch that enables it, so it reports BKPT 0xAB as an ordinary
            # BKPT and leaves the PC on the instruction.  Recognise the trap
            # here, service it, and step over it ourselves.
            self._semihost.handle(box, self._mem, self._reg)
            # ``pc_t`` keeps bit 0 set: a plain PC write would drop the core into
            # Arm state and the next fetch would be an invalid instruction.
            self._reg.pc_t = (self._reg.pc & ~0x1) + 2

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
        """Peripheral model registry for a core/mode plus a vendor family."""
        if mode != "cortex_m":
            raise XwUnknownHardware(f"Unknown Arm core: {mode}")
        # Imported lazily: cortex_m/__init__ imports this module.
        from . import CORE_PERIPHERALS

        buildin = dict(CORE_PERIPHERALS)
        # name
        if name == "sam":
            from ..vendor.atmel import BUILDIN
            buildin.update(BUILDIN)
        elif name == "stm":
            from ..vendor.st import BUILDIN
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
                # create instance; caller supplied options (e.g. the serial bridge)
                # override the chip description
                spec = dict(device[name])
                spec.update(self._options)
                buildin_ = buildin[class_](self._box, self, name, **spec)
                self.perif[name_] = buildin_
                self._mem.register_io(base, size, self.perif[name_].read, self.perif[name_].write, name)
                if hasattr(buildin_, "system_clock_callback"):
                    logger.debug(f"[{name:8s}]: Add system_clock_callback")
                    self._box.hook_add(UC_HOOK_CODE, buildin_.system_clock_callback)
                if "dma_base" in device[name]:
                    dma_name_ = f"{name_}-dma"
                    dma_name = f"{name}-DMA"
                    dma_base = device[name]["dma_base"]
                    dma_size = device[name]["dma_size"]
                    self.perif[dma_name_] = buildin["dma"](self._box, self, dma_name, dma_base, dma_size)
                    self._mem.register_io(
                        dma_base, dma_size, self.perif[dma_name_].read, self.perif[dma_name_].write, dma_name
                    )
                    self.perif[name_].dma = self.perif[dma_name_]

    def reset(self):
        for name, buildin in self.perif.items():
            logger.debug(f"Reset {name}")
            buildin.reset()
