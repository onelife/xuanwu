# -*- coding: utf-8 -*-

"""Host-facing back ends.

Everything that depends on the host operating system lives here, so the
simulation core stays portable.
"""

from .semihost import SEMIHOST_BKPT, SemiHosting, SemiHostingOp
from .serial_bridge import (
    BRIDGE_KINDS,
    LoopbackBridge,
    NullBridge,
    SerialBridge,
    SocatBridge,
    TcpBridge,
    create_bridge,
)

__all__ = [
    "SerialBridge",
    "SocatBridge",
    "TcpBridge",
    "LoopbackBridge",
    "NullBridge",
    "create_bridge",
    "BRIDGE_KINDS",
    "SemiHosting",
    "SemiHostingOp",
    "SEMIHOST_BKPT",
]
