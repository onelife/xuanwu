# -*- coding: utf-8 -*-

"""Architecture support.

``base`` holds the register-table foundation shared by every peripheral model,
and ``cortex_m`` holds the Armv7-M core peripherals plus the interrupt engine.
Vendor families live under ``vendor``.
"""

from .base import *  # noqa: F401,F403
from .base import __all__ as _base_all
from .cortex_m import *  # noqa: F401,F403
from .cortex_m import __all__ as _cortex_all

__all__ = list(_base_all) + list(_cortex_all)
