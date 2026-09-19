# -*- coding: utf-8 -*-

"""Atmel SAM3X peripheral family.

The models now live one-per-file under :mod:`xuanwu.arch.vendor.atmel`; this
module keeps the old import path working.
"""

import warnings

from .vendor.atmel import *  # noqa: F401,F403
from .vendor.atmel import __all__ as _atmel_all

__all__ = list(_atmel_all)

warnings.warn(
    "xuanwu.arch.atmel_sam has moved to xuanwu.arch.vendor.atmel; import from there instead.",
    DeprecationWarning,
    stacklevel=2,
)
