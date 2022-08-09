# -*- coding: utf-8 -*-

from enum import IntEnum


class EXCP(IntEnum):
    UDEF = 1  # undefined instruction
    SWI = 2  # software interrupt
    PREFETCH_ABORT = 3
    DATA_ABORT = 4
    IRQ = 5
    FIQ = 6
    BKPT = 7
    EXCEPTION_EXIT = 8  # Return from v7M exception
    KERNEL_TRAP = 9  # Jumped to kernel code page
    HVC = 11  # HyperVisor Call
    HYP_TRAP = 12
    SMC = 13  # Secure Monitor Call
    VIRQ = 14
    VFIQ = 15
    SEMIHOST = 16  # semihosting call
    NOCP = 17  # v7M NOCP UsageFault
    INVSTATE = 18  # v7M INVSTATE UsageFault
    STKOF = 19  # v8M STKOF UsageFault
    LAZYFP = 20  # v7M fault during lazy FP stacking
    LSERR = 21  # v8M LSERR SecureFault
    UNALIGNED = 22  # v7M UNALIGNED UsageFault
