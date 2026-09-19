# -*- coding: utf-8 -*-

"""Values shared by several SAM3X peripherals."""

from enum import IntEnum

__all__ = ["PID"]


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
