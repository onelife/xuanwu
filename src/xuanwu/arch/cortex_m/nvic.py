# -*- coding: utf-8 -*-

"""Nested vectored interrupt controller."""

from functools import partial
from typing import Any, Optional, Set, Tuple

from ...config import logger
from ..base import ArmHardwareBase, IrqOp, Register

__all__ = ["ArmHardwareNvic"]


class ArmHardwareNvic(ArmHardwareBase):
    """nested vectored interrupt controller"""

    NAME = "NVIC"
    # The register table depends on ``interrupt_lines``/``priority_bits``, so it
    # is expanded per instance from this template.  It used to be assigned to the
    # class from __init__, which made two chips with different line counts in one
    # process overwrite each other's table.
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
        # Interrupt Clear-pending Registers.  Write-only: a write clears the bit in
        # the mirrored ISPR and the register itself stays zero (fix_before_write
        # returns 0), and a read gives ISPR back.  The write mask used to be zero,
        # which turned every NVIC_ClearPendingIRQ() into a no-op.
        ("ICPR{0}", "I", 0xFFFFFFFF),
        ("RESERVED3", "{0}I", 0x00000000),
        # Interrupt Active Bit Registers
        ("IABR{0}", "I", 0x00000000),
        # The gap after IABR is one 32-word block longer than the others, because the
        # priority registers sit a block further along than a naive "one reserved gap
        # per block" layout puts them: IPR0 is at +0x300 from ISER0 (Armv7-M 4.2.2),
        # not +0x280.  With the short gap a driver's NVIC_SetPriority() -- which
        # writes 0xE000E400, the architectural IPR0 -- landed on IPR32 instead, so the
        # priority the guest set was not the priority the model scheduled on.
        ("RESERVED4", "{0}I", 0x00000000),
        ("RESERVED5", "32I", 0x00000000),
        ("IPR{0}", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        line_num = kwargs.get("interrupt_lines", 7) + 1
        priority_bits = kwargs.get("priority_bits", 4)
        # Only the top `priority_bits` of each priority byte are implemented: the low
        # bits read as zero and ignore writes (Armv7-M 4.2.3).  The mask used to be
        # `~((1 << priority_bits) - 1) & 0xFF`, which keeps the *low* bits and throws
        # the implemented ones away -- the same value as the architecture only for the
        # four-bit parts every description declares.
        priority_mask = (0xFF << (8 - priority_bits)) & 0xFF
        mask = 0
        for _ in range(4):
            mask = (mask << 8) | priority_mask
        REGS = []
        for name, fmt, write_mask in ArmHardwareNvic.REGISTERS_TEMPLATE:
            if name.startswith("RESERVED"):
                # A gap fills the rest of its 32-word block; a literal size (the extra
                # block above) is used as written.
                size = fmt.format(32 - line_num) if "{0}" in fmt else fmt
                REGS.extend([(name, size, write_mask)])
            elif name.startswith("IPR"):
                REGS.extend([(name.format(x), fmt, mask) for x in range((line_num * 32 - 16) // 4)])
            else:
                REGS.extend([(name.format(x), fmt, write_mask) for x in range(line_num)])
        # instance attribute: ArmHardwareBase.__init__ reads it through self
        self.REGISTERS = tuple(REGS)
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
    def _block_index(name: str) -> int:
        """The index in a register name: ``ISER10`` -> 10, ``IPR3`` -> 3.

        The callers used ``int(name[-1])``, which reads ``ISER10`` as ``ISER0`` -- out
        of reach with the eight words a 240-interrupt part needs, and wrong the moment
        a description declares more than 320 lines.
        """
        return int(name.lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ"))

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
            ispr |= 1 << offset
        else:
            ispr &= ~(1 << offset)
        self.write_register(f"ISPR{reg_num}", ispr)

    def set_active(self, irq: int, state: Optional[bool] = False) -> None:
        if irq < 0 or irq >= 240:
            logger.error(f"[{self.NAME:8s}]: Invalid IRQ when set active, {irq}")
            raise RuntimeError(f"Invalid IRQ when set active, {irq}")
        offset, reg_num = self._irq2reg(irq)
        iabr = self.read_register(f"IABR{reg_num}")
        if state:
            iabr |= 1 << offset
        else:
            iabr &= ~(1 << offset)
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
            # ICER/ICPR are write-only; reading them returns ISER/ISPR.
            data = self.read_register("IS" + name[2:])
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name.startswith("ISER") or name.startswith("ISPR"):
            data = data_orig | data
            if name.startswith("ISPR"):
                set_bits = (data ^ data_orig) & data
                irqs = self._reg2irq(set_bits, self._block_index(name))
                for irq in irqs:
                    self._ctl.set_irq_pending(irq)
        elif name.startswith("ICER") or name.startswith("ICPR"):
            # ICER/ICPR clear bits in the mirrored ISER/ISPR registers.
            reg_ = "IS" + name[2:]
            data_orig_ = self.read_register(reg_)
            data_ = data_orig_ & ~data
            self.write_register(reg_, data_)
            if name.startswith("ICPR"):
                clear_bits = (~data ^ data_orig_) & data
                irqs = self._reg2irq(clear_bits, self._block_index(name))
                for irq in irqs:
                    self._ctl.clear_irq_pending(irq)
            data = 0
        elif name.startswith("IABR"):
            logger.warning(f"[{name_:16s}]: {name} is read-only")
        elif name == "STIR":
            # When the USERSETMPEND bit in the SCR is set to 1, unprivileged software can access
            logger.debug(f"[{name_:16s}]: Software trigger IRQ_{data}")
        return data
