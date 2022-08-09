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
