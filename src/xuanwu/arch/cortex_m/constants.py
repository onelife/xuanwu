# -*- coding: utf-8 -*-

"""Armv7-M architectural constants (exception numbers and core register bits)."""

from enum import IntEnum

__all__ = [
    "Exception_",
    "CONTROL",
    "EPSR",
    "ICSR",
    "CCR",
    "CFSR",
    "SHCSR",
    "CSR",
]


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


# # Software Trigger Interrupt Register
# (("STIR", "I", 0x000001FF),)


class CSR(IntEnum):
    ENABLE = 0
    TICKINT = 1
    CLKSOURCE = 2
    COUNTFLAG = 16


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
