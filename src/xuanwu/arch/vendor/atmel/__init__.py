# -*- coding: utf-8 -*-

"""Atmel SAM3X peripherals."""

from .adc import ArmSamAdc
from .common import PID
from .efc import ArmSamEfc
from .pdc import ArmSamPdc
from .pio import ArmSamGpio
from .pmc import ArmSamPmc
from .pwm import ArmSamPwm
from .spi import ArmSamSpi
from .twi import ArmSamTwi
from .uart import ArmSamUart
from .uotghs import ArmSamUotghs

__all__ = [
    "PID",
    "ArmSamPmc",
    "ArmSamPdc",
    "ArmSamEfc",
    "ArmSamGpio",
    "ArmSamAdc",
    "ArmSamUart",
    "ArmSamSpi",
    "ArmSamTwi",
    "ArmSamPwm",
    "ArmSamUotghs",
    "BUILDIN",
]


BUILDIN = {
    "pmc": ArmSamPmc,
    "dma": ArmSamPdc,
    "efc": ArmSamEfc,
    "gpio": ArmSamGpio,
    "adc": ArmSamAdc,
    "pwm": ArmSamPwm,
    "uart": ArmSamUart,
    "spi": ArmSamSpi,
    "twi": ArmSamTwi,
    "uotghs": ArmSamUotghs,
}
"""Peripheral models keyed by the name used in the chip YAML."""
