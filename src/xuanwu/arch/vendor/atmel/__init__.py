# -*- coding: utf-8 -*-

"""Atmel SAM3X peripherals."""

from .adc import ArmSamAdc
from .common import PID
from .pdc import ArmSamPdc
from .pio import ArmSamGpio
from .pmc import ArmSamPmc
from .pwm import ArmSamPwm
from .spi import ArmSamSpi
from .uart import ArmSamUart
from .uotghs import ArmSamUotghs

__all__ = [
    "PID",
    "ArmSamPmc",
    "ArmSamPdc",
    "ArmSamGpio",
    "ArmSamAdc",
    "ArmSamUart",
    "ArmSamSpi",
    "ArmSamPwm",
    "ArmSamUotghs",
    "BUILDIN",
]


BUILDIN = {
    "pmc": ArmSamPmc,
    "dma": ArmSamPdc,
    "gpio": ArmSamGpio,
    "adc": ArmSamAdc,
    "pwm": ArmSamPwm,
    "uart": ArmSamUart,
    "spi": ArmSamSpi,
    "uotghs": ArmSamUotghs,
}
"""Peripheral models keyed by the name used in the chip YAML."""
