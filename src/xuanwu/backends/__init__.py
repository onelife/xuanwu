# -*- coding: utf-8 -*-

"""Host-facing back ends.

Everything that depends on the host operating system lives here, so the
simulation core stays portable.
"""

from .serial_bridge import (
    BRIDGE_KINDS,
    LoopbackBridge,
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
    "create_bridge",
    "BRIDGE_KINDS",
]
