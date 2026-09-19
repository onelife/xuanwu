# -*- coding: utf-8 -*-

"""Universal asynchronous receiver transceiver."""

from enum import IntEnum
from typing import Any, Optional

from unicorn import Uc

from ....backends import create_bridge
from ....config import logger
from ...base import ArmHardwareBase, Register
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
        self._bridge = create_bridge(
            kwargs.get("bridge", "auto"),
            baudrate=kwargs.get("baudrate", 115200),
            prefix="uart",
        )
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

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        cr = self.read_register("CR")
        sr = data = self.read_register("SR")
        if self._bridge.in_waiting > 0:
            if cr & (1 << UART_CR.RXEN):
                if not self._last_rx_new:
                    # self._last_rx = int.from_bytes(self._bridge.read(self._bridge.in_waiting + 10)[-1], "little")
                    rx = self._bridge.read(1)
                    self._last_rx = int.from_bytes(rx, "little")
                    # logger.debug(f"UART RX: {rx}")
                    self._last_rx_new = True
                data |= 1 << UART_SR.RXRDY
            else:
                _ = self._bridge.read(self._bridge.in_waiting + 128)
        # if self._bridge.in_waiting > 1:
        #     sr |= 1 << UART_SR.OVRE
        if cr & (1 << UART_CR.TXEN):
            # if self._bridge.out_waiting <= 1:
            #     sr |= 1 << UART_SR.TXRDY
            data |= 1 << UART_SR.TXRDY
        if sr != data:
            self.write_register("SR", data)
        imr = self.read_register("IMR")
        if imr & data and not self._ctl.is_irq_pending_or_active(self._irq):
            self._ctl.set_irq_pending(self._irq)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "RHR":
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
