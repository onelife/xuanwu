# -*- coding: utf-8 -*-

from typing import Any


__all__ = [
    "XwInvalidParameter",
    "XwInvalidChipInformation",
    "XwInvalidCodeFormat",
    "XwInvalidCodeFile",
    "XwInvalidMemoryAddress",
    "XwInvalidMemorySize",
    "XwUnknownHardware",
    "XwUnsupported",
    "XwSerialBridgeError",
]


class BaseException(Exception):
    def __init__(self, message: str, **kwargs: Any):
        super().__init__(**kwargs)
        self.message = message

    def __str__(self):
        return self.message


class XwInvalidParameter(BaseException):
    pass


class XwInvalidChipInformation(BaseException):
    pass


class XwInvalidCodeFormat(BaseException):
    pass


class XwInvalidCodeFile(BaseException):
    pass


class XwInvalidMemoryAddress(BaseException):
    pass


class XwInvalidMemorySize(BaseException):
    pass


class XwUnknownHardware(BaseException):
    pass


class XwUnsupported(BaseException):
    """Raised when the guest asks for something the simulator does not model.

    A core exception the engine has no behaviour for (a BKPT that is not the
    semihosting trap, say) ends the run with this, rather than with a bare ``raise``
    whose message -- "No active exception to reraise" -- says nothing about what the
    guest did.
    """


class XwSerialBridgeError(BaseException):
    """Raised when a peripheral's host serial bridge (e.g. socat) cannot be set up."""
