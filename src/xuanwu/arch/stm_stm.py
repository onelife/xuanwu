# -*- coding: utf-8 -*-

"""STMicroelectronics STM32 peripheral family.

The models now live one-per-file under :mod:`xuanwu.arch.vendor.st`; this module
keeps the old import path working.
"""

import warnings

from .vendor.st import *  # noqa: F401,F403
from .vendor.st import __all__ as _st_all

__all__ = list(_st_all)

warnings.warn(
    "xuanwu.arch.stm_stm has moved to xuanwu.arch.vendor.st; import from there instead.",
    DeprecationWarning,
    stacklevel=2,
)
