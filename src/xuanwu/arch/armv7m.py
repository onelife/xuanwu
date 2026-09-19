# -*- coding: utf-8 -*-

"""Armv7-M support: the Cortex-M core peripherals plus the generic peripheral base.

This module used to hold everything in one 1266-line file.  The code now lives in
:mod:`xuanwu.arch.base` and :mod:`xuanwu.arch.cortex_m`; the old import path keeps
working here for backwards compatibility.
"""

import warnings

from .base import *  # noqa: F401,F403
from .base import __all__ as _base_all
from .cortex_m import *  # noqa: F401,F403
from .cortex_m import __all__ as _cortex_all

__all__ = list(_base_all) + list(_cortex_all)

warnings.warn(
    "xuanwu.arch.armv7m has been split into xuanwu.arch.base and xuanwu.arch.cortex_m; "
    "import from those modules instead.",
    DeprecationWarning,
    stacklevel=2,
)
