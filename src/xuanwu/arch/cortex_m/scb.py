# -*- coding: utf-8 -*-

"""System control block."""

from functools import partial
from typing import Any, Optional

from ...config import logger
from ..base import ArmHardwareBase, IrqOp, Register
from .constants import ICSR, SHCSR, Exception_

__all__ = ["ArmHardwareScb"]


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
        self.write_register("CPUID", self._cpuid)
        self.write_register("ICSR", 0x0)
        self.write_register("VTOR", 0x0)
        self.write_register("AIRCR", 0xFA050000)
        # SCR only defines bits 1 (SLEEPONEXIT), 2 (SLEEPDEEP), 4 (SEVONPEND) and 8 (USERSETMPEND).
        self.write_register("SCR", 0x0)
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
