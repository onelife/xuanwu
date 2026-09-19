# -*- coding: utf-8 -*-

"""A minimal SPI NOR flash.

Wired to an SPI port it *becomes* that peripheral's byte stream, so every byte
the firmware writes to the transmit register is executed as a flash command and
every response is queued for the receive register.  A GPIO pin named by ``cs``
delimits transactions, exactly as the chip select does on real hardware.

Implemented: JEDEC ID, read status, write enable/disable, read data, page
program and the three erase commands.  Anything else is ignored, which is what a
real part does with an unknown opcode.
"""

import threading
from enum import IntEnum
from typing import Any, Dict, Optional

from ..backends.serial_bridge import SerialBridge
from ..config import logger
from .base import Device, DeviceContext

__all__ = ["SpiFlash", "SpiFlashCommand"]


class SpiFlashCommand(IntEnum):
    PAGE_PROGRAM = 0x02
    READ_DATA = 0x03
    WRITE_DISABLE = 0x04
    READ_STATUS = 0x05
    WRITE_ENABLE = 0x06
    SECTOR_ERASE = 0x20
    BLOCK_ERASE = 0xD8
    CHIP_ERASE = 0x60
    CHIP_ERASE_ALT = 0xC7
    JEDEC_ID = 0x9F
    RELEASE_POWER_DOWN = 0xAB


ADDRESS_COMMANDS = frozenset(
    {
        SpiFlashCommand.PAGE_PROGRAM,
        SpiFlashCommand.READ_DATA,
        SpiFlashCommand.SECTOR_ERASE,
        SpiFlashCommand.BLOCK_ERASE,
    }
)

STATUS_WEL = 0x02
"""Write-enable latch."""

ERASED = 0xFF


class SpiFlash(Device, SerialBridge):
    """A serial NOR flash that answers the commands firmware actually uses."""

    type = "spi_flash"
    kind = "spi_flash"

    # Winbond W25Q128 style identification.
    MANUFACTURER = 0xEF
    MEMORY_TYPE = 0x40
    CAPACITY_CODE = 0x18

    def __init__(
        self,
        name: str,
        port: str = "SPI",
        cs: Optional[Dict[str, Any]] = None,
        size: int = 0x100000,
        manufacturer: int = MANUFACTURER,
        memory_type: int = MEMORY_TYPE,
        capacity_code: int = CAPACITY_CODE,
        **options: Any,
    ) -> None:
        super().__init__(name, **options)
        self.port = port
        self.cs = dict(cs or {})
        self.size = int(size)
        self.manufacturer = int(manufacturer)
        self.memory_type = int(memory_type)
        self.capacity_code = int(capacity_code)

        self._memory = bytearray([ERASED]) * self.size
        self._status = 0x00
        self._selected = False
        self._command: Optional[int] = None
        self._address = 0
        self._address_bytes = 0
        self._pending_write = False
        self._rx = bytearray()
        self._lock = threading.RLock()
        self._peripheral = None
        self._gpio = None
        self._cs_pin: Optional[int] = None
        self._cs_active_low = True

    # -- content access (tests, preloading) ------------------------------

    def read_memory(self, address: int, size: int) -> bytes:
        with self._lock:
            return bytes(self._memory[address : address + size])

    def write_memory(self, address: int, data: bytes) -> None:
        with self._lock:
            self._memory[address : address + len(data)] = data

    # -- Device ----------------------------------------------------------

    def attach(self, ctx: DeviceContext) -> None:
        peripheral = ctx.peripheral(self.port)
        peripheral.bridge = self  # take over the bus
        self._peripheral = peripheral

        if self.cs:
            self._gpio = ctx.peripheral(self.cs["port"])
            self._cs_pin = int(self.cs["pin"])
            self._cs_active_low = bool(self.cs.get("active_low", True))
            self._gpio.add_hook(self._cs_pin, (self._on_cs_high, self._on_cs_low))
        else:
            logger.warning(
                f"[{self.name:8s}]: no 'cs' pin configured -- transactions cannot be delimited, "
                "so consecutive commands will run together"
            )

        self.attached = True
        chip_select = f", CS {self.cs['port']}.{self._cs_pin}" if self._cs_pin is not None else ""
        logger.info(f"[{self.name:8s}]: SPI flash on {self.port}, 0x{self.size:x} bytes{chip_select}")

    def detach(self) -> None:
        if self._gpio is not None and self._cs_pin is not None:
            self._gpio.remove_hook(self._cs_pin, (self._on_cs_high, self._on_cs_low))
        self._gpio = None
        self._peripheral = None
        super().detach()

    def reset(self) -> None:
        with self._lock:
            self._status = 0x00
            self._selected = False
            self._command = None
            self._address = 0
            self._address_bytes = 0
            self._pending_write = False
            self._rx.clear()

    # -- chip select -----------------------------------------------------

    def _on_cs_high(self) -> None:
        self.select(not self._cs_active_low)

    def _on_cs_low(self) -> None:
        self.select(self._cs_active_low)

    def select(self, active: bool) -> None:
        """Assert or release chip select, delimiting one transaction.

        Normally driven by the ``cs`` GPIO pin; call it directly when the flash
        is wired without one.
        """
        if active == self._selected:
            return
        self._selected = active
        if active:
            # a new transaction: the next byte is a command
            self._command = None
            self._address = 0
            self._address_bytes = 0
        else:
            self._command = None
            if self._pending_write:
                # a real part clears the write-enable latch when the
                # program/erase operation completes, i.e. when CS is released
                self._status &= ~STATUS_WEL
                self._pending_write = False

    # -- SerialBridge: the peripheral's end of the bus -------------------

    @property
    def peer_hint(self) -> str:
        return f"device://{self.name}"

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            chunk = bytes(self._rx[:size])
            del self._rx[:size]
        return chunk

    def write(self, data: bytes) -> int:
        for value in data:
            self._execute(value)
        return len(data)

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        pass

    # -- command state machine -------------------------------------------

    def _emit(self, data: bytes) -> None:
        with self._lock:
            self._rx.extend(data)

    def _execute(self, value: int) -> None:
        with self._lock:
            if self._command is None:
                self._command = value
                self._address = 0
                self._address_bytes = 0
                self._begin(value)
                return

            if self._command in ADDRESS_COMMANDS and self._address_bytes < 3:
                self._address = ((self._address << 8) | value) & 0xFFFFFF
                self._address_bytes += 1
                if self._address_bytes == 3:
                    self._address %= self.size
                    if self._command in (SpiFlashCommand.SECTOR_ERASE, SpiFlashCommand.BLOCK_ERASE):
                        self._erase_command(self._command)
                return

            if self._command == SpiFlashCommand.READ_DATA:
                self._rx.append(self._memory[self._address])
                self._address = (self._address + 1) % self.size
            elif self._command == SpiFlashCommand.PAGE_PROGRAM:
                if self._status & STATUS_WEL:
                    self._memory[self._address] = value
                    self._pending_write = True
                self._address = (self._address + 1) % self.size

    def _begin(self, command: int) -> None:
        if command == SpiFlashCommand.JEDEC_ID:
            self._emit(bytes([self.manufacturer, self.memory_type, self.capacity_code]))
        elif command == SpiFlashCommand.READ_STATUS:
            self._emit(bytes([self._status]))
        elif command == SpiFlashCommand.WRITE_ENABLE:
            self._status |= STATUS_WEL
        elif command == SpiFlashCommand.WRITE_DISABLE:
            self._status &= ~STATUS_WEL
        elif command == SpiFlashCommand.RELEASE_POWER_DOWN:
            # three dummy bytes, then the device id
            self._emit(b"\x00\x00\x00" + bytes([self.memory_type]))
        elif command in (SpiFlashCommand.CHIP_ERASE, SpiFlashCommand.CHIP_ERASE_ALT):
            self._erase(0, self.size)
        elif command in ADDRESS_COMMANDS:
            pass  # the address bytes follow
        else:
            logger.debug(f"[{self.name:8s}]: ignoring opcode 0x{command:02x}")

    def _erase_command(self, command: int) -> None:
        if command == SpiFlashCommand.SECTOR_ERASE:
            self._erase(self._address & ~0xFFF, 0x1000)
        else:
            self._erase(self._address & ~0xFFFF, 0x10000)

    def _erase(self, start: int, size: int) -> None:
        if not self._status & STATUS_WEL:
            logger.debug(f"[{self.name:8s}]: erase ignored, write enable not set")
            return
        end = min(start + size, self.size)
        self._memory[start:end] = bytes([ERASED]) * (end - start)
        self._pending_write = True
        logger.debug(f"[{self.name:8s}]: erased 0x{start:06x}..0x{end:06x}")
