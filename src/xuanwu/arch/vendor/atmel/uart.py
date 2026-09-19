# -*- coding: utf-8 -*-

"""Universal asynchronous receiver transceiver."""

from enum import IntEnum
from typing import Any

from ....backends import create_bridge
from ....config import logger
from ...base import NEVER, ArmHardwareBase, Register
from .common import PID
from .pdc import ArmSamPdc

__all__ = ["ArmSamUart"]


class UART_CR(IntEnum):
    RSTRX = 2
    RSTTX = 3
    RXEN = 4
    RXDIS = 5
    TXEN = 6
    TXDIS = 7
    RSTSTA = 8


class UART_SR(IntEnum):
    RXRDY = 0
    TXRDY = 1
    ENDRX = 3
    ENDTX = 4
    OVRE = 5
    FRAME = 6
    PARE = 7
    TXEMPTY = 9
    TXBUFE = 11
    RXBUFF = 12


class ArmSamUart(ArmHardwareBase):
    """Universal Asynchronous Receiver Transceiver"""

    NAME = "UART"
    REGISTERS = (
        ("CR", "I", 0xFFFFFFFF),
        ("MR", "I", 0xFFFFFFFF),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("SR", "I", 0x00000000),
        ("RHR", "I", 0x00000000),
        ("THR", "I", 0xFFFFFFFF),
        ("BRGR", "I", 0xFFFFFFFF),
    )
    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._irq = PID.UART
        self._last_rx = 0
        self._last_rx_new = False
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # 'bridge' may be set in the chip YAML: auto | socat | tcp | loopback
        self.baudrate = int(kwargs.get("baudrate", 115200))
        self._bridge = create_bridge(
            kwargs.get("bridge", "auto"),
            baudrate=self.baudrate,
            prefix="uart",
        )
        # Polling the host faster than a character can arrive buys nothing, and every
        # poll ends an execution slice, so it costs Python work on the hot path: one
        # character time is the interval at which an incoming byte is seen as soon as
        # the line could have delivered it.  Without a clock (a hand-written
        # peripheral) fall back to a slice that is long but still responsive.
        clock = int(kwargs.get("clock", 0))
        if clock and self.baudrate:
            self.poll_interval = max(256, clock * 10 // self.baudrate)
        else:
            self.poll_interval = 4096
        logger.info(f"Serial device (UART): {self._bridge.peer_hint}")
        self._dma = None

    def __del__(self):
        bridge = getattr(self, "_bridge", None)
        if bridge is not None:
            bridge.close()

    @property
    def dma(self) -> ArmSamPdc:
        return self._dma

    @dma.setter
    def dma(self, dma: ArmSamPdc) -> None:
        self._dma = dma

    def reset(self):
        super().reset()
        self.write_register("MR", 0x00000000)
        self.write_register("IMR", 0x00000000)
        self.write_register("SR", 0x00000000)
        self.write_register("RHR", 0x00000000)
        self.write_register("BRGR", 0x00000000)

    @property
    def peer_hint(self) -> str:
        """Where an external program should attach (a pty path, or tcp://host:port)."""
        return self._bridge.peer_hint

    @property
    def bridge(self):
        """The other end of this peripheral's byte stream."""
        return self._bridge

    @bridge.setter
    def bridge(self, bridge) -> None:
        # Lets the device layer take over the line (or hand it back).
        previous = getattr(self, "_bridge", None)
        if previous is not None:
            previous.close()
        self._bridge = bridge

    POLL_INTERVAL = 512
    """Fallback instructions between polls of the host side (see ``poll_interval``).

    The byte stream from the bridge arrives asynchronously, so the poll interval is
    how long a byte may sit unnoticed -- it also bounds how long the scheduler may
    run.  The instance value is derived from the peripheral clock and the baud rate
    when the chip description declares a clock, and this is what is used otherwise.
    """

    def refresh_status(self) -> None:
        """Sample the host side and update the status/interrupt state."""
        cr = self.read_register("CR")
        sr = data = self.read_register("SR")
        if self._bridge.in_waiting > 0:
            if cr & (1 << UART_CR.RXEN):
                if not self._last_rx_new:
                    rx = self._bridge.read(1)
                    self._last_rx = int.from_bytes(rx, "little")
                    self._last_rx_new = True
                data |= 1 << UART_SR.RXRDY
            else:
                _ = self._bridge.read(self._bridge.in_waiting + 128)
        if cr & (1 << UART_CR.TXEN):
            data |= 1 << UART_SR.TXRDY
        if sr != data:
            self.write_register("SR", data)
        imr = self.read_register("IMR")
        if imr & data and not self._ctl.is_irq_pending_or_active(self._irq):
            self._ctl.set_irq_pending(self._irq)

    def advance(self, instructions: int) -> None:
        """Called at execution-slice boundaries instead of once per instruction."""
        self.refresh_status()

    def next_deadline(self) -> int:
        # Polling the host is only worth anything while the receiver is on.
        if self.read_register("CR") & (1 << UART_CR.RXEN):
            return self.poll_interval
        return NEVER

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "SR":
            # The host side can deliver a byte at any time, so sample it here
            # rather than relying on the slice boundary.
            self.refresh_status()
        elif name == "RHR":
            sr = data_ = self.read_register("SR")
            data_ &= ~(1 << UART_SR.RXRDY)
            if sr != data_:
                self.write_register("SR", data_)
            data = self._last_rx
            self._last_rx_new = False
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            sr = self.read_register("SR")
            sr_orig = sr
            if data & ((1 << UART_CR.RSTRX) | (1 << UART_CR.RXDIS)):
                sr &= ~(1 << UART_SR.RXRDY)
                if data & (1 << UART_CR.RSTRX):
                    self._bridge.reset_input_buffer()
            if data & ((1 << UART_CR.RSTTX) | (1 << UART_CR.TXDIS)):
                sr &= ~(1 << UART_SR.TXRDY)
                if data & (1 << UART_CR.RSTTX):
                    self._bridge.reset_output_buffer()
            if data & (1 << UART_CR.RSTSTA):
                sr &= ~((1 << UART_SR.OVRE) | (1 << UART_SR.FRAME) | (1 << UART_SR.PARE))
            self.write_register("SR", sr)
            diff = sr_orig ^ sr
            if diff & (1 << UART_SR.RXRDY):
                if sr & (1 << UART_SR.RXRDY):
                    logger.debug(f"[{name_:16s}]: Enable RX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable RX")
            if diff & (1 << UART_SR.TXRDY):
                if sr & (1 << UART_SR.TXRDY):
                    logger.debug(f"[{name_:16s}]: Enable TX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable TX")
        elif name in ["IER"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            data = 0
        elif name in ["IDR"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val & ~data
            self.write_register(reg, new_val)
        elif name == "THR":
            cr = self.read_register("CR")
            if cr & (1 << UART_CR.TXEN):
                # if self._bridge.out_waiting <= 1:
                #     self._bridge.write(data.to_bytes(1, "little"))
                self._bridge.write((data & 0xFF).to_bytes(1, "little"))
                # logger.debug(f'[{name_:16s}]: Output "{(data & 0xFF).to_bytes(1, "little")}"')
        return data
