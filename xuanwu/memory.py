# -*- coding: utf-8 -*-

# from os import path, fstat
from collections import namedtuple
from struct import Struct
from typing import Optional, Any, Dict, List, Tuple, Set, Iterator, Callable

from unicorn import Uc, UC_PROT_ALL, UC_PROT_READ, UC_PROT_WRITE, UC_PROT_EXEC

# from unicorn import UC_PROT_NONE, UC_PROT_READ, UC_PROT_WRITE, UC_PROT_EXEC, UC_PROT_ALL

from .config import logger
from .exception import XwInvalidParameter, XwInvalidMemoryAddress, XwInvalidMemorySize


__all__ = ["MemoryController"]


MemoryInfo = namedtuple("Memory", ["start", "end", "perms", "buffer", "desc"])
MemoryIoInfo = namedtuple("IO", ["start", "end", "read_fn", "write_fn", "desc"])


class MemoryController(object):
    """Memory control unit"""

    size2fmt = {
        1: "B",
        2: "H",
        4: "I",
        # 8: "Q",
    }

    def __init__(self, box: Uc, little_endian: bool, is_64bit: bool, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._box = box
        self._endian_str = "little" if little_endian else "big"
        self._endian = "<" if little_endian else ">"
        self._is_64bit = is_64bit
        self._word = "Q" if is_64bit else "I"
        self._registry: List[MemoryInfo] = []
        self._io_registry: List[MemoryIoInfo] = []

    def get_map(self, start: int, end: int, no_buf_only: Optional[bool] = False) -> List[MemoryInfo]:
        ret = []
        for record in self._registry:
            if no_buf_only and record.buffer:
                continue
            if (record.start <= start < record.end) or (record.start < end <= record.end):
                ret.append(record)
        return ret

    def get_io(self, start: int, end: int) -> List[MemoryIoInfo]:
        ret = []
        for record in self._io_registry:
            if (record.start <= start < record.end) or (record.start < end <= record.end):
                ret.append(record)
        return ret

    def map_iterator(self) -> Iterator[MemoryInfo]:
        for item in sorted(self._registry, key=lambda x: x.start):
            yield item

    def io_iterator(self) -> Iterator[MemoryIoInfo]:
        for item in sorted(self._io_registry, key=lambda x: x.start):
            yield item

    def register_io(
        self,
        address: int,
        size: int,
        read_fn: Optional[Callable] = None,
        write_fn: Optional[Callable] = None,
        desc: Optional[str] = None,
    ) -> None:
        records = self.get_map(address, address + size)
        if not records:
            raise XwInvalidMemoryAddress(f"Unmapped memory address: 0x{address:08X} (0x{size:08X})")
        if len(records) > 1:
            raise XwInvalidMemoryAddress(f"Invalid memory IO address to read: 0x{address:08X} (0x{size:08X})")
        logger.debug(f"register_io: 0x{address:08X} (0x{size:08X}) [{desc}]")
        self._io_registry.append(MemoryIoInfo(address, address + size, read_fn, write_fn, desc or "Mapped IO"))

    def show_map(self) -> None:
        from collections import OrderedDict

        perms2str = OrderedDict({UC_PROT_READ: "r", UC_PROT_WRITE: "w", UC_PROT_EXEC: "x"})
        print(f'\n{"Start":10s}   {"End":10s}   {"Permission":10s}   {"Description":32s}')
        for record in self.map_iterator():
            print(
                f"0x{record.start:08x} ~ 0x{record.end:08x}   "
                f'{"".join([v if (k & record.perms) else "-" for k,v in perms2str.items()]):10s}   '
                f"[{record.desc:32s}]"
            )
        print(f'\n{"Start":10s}   {"End":10s}   {"Read/Write":10s}   {"Description":32s}')
        for record in self.io_iterator():
            print(
                f"0x{record.start:08x} ~ 0x{record.end:08x}   "
                f'{("r" if record.read_fn else "-") + ("w" if record.write_fn else "-"):10s}   '
                f"[{record.desc:32s}]"
            )

    def map(
        self,
        address: int,
        size: int,
        perms: Optional[int] = UC_PROT_ALL,
        desc: Optional[str] = None,
    ) -> None:
        # check parameters
        if address < 0:
            raise XwInvalidParameter(f"Invalid memory address: {address}")
        if size <= 0:
            raise XwInvalidParameter(f"Invalid memory size: {size}")
        if not (0 <= perms <= UC_PROT_ALL):
            raise XwInvalidParameter(f"Invalid memory permission: {perms}")
        # check if new entry
        if self.get_map(address, address + size):
            raise XwInvalidMemoryAddress("Already mapped memory address: 0x{address:08X} (0x{size:08X})")
        # do map and add entry
        logger.debug(f"map: 0x{address:08x} (0x{size:08x}) [{desc}]")
        self._box.mem_map(address, size, perms)
        self._registry.append(MemoryInfo(address, address + size, perms, None, desc or "Mapped"))

    def map_with_callback(
        self,
        address: int,
        size: int,
        buffer: bytearray,
        read_cb: Callable,
        user_data_read: Any,
        write_cb: Callable,
        user_data_write: Any,
        perms: Optional[int] = UC_PROT_ALL,
        desc: Optional[str] = None,
    ) -> None:
        # check parameters
        if address < 0:
            raise XwInvalidParameter(f"Invalid memory address: {address}")
        if size <= 0:
            raise XwInvalidParameter(f"Invalid memory size: {size}")
        if not (0 <= perms <= UC_PROT_ALL):
            raise XwInvalidParameter(f"Invalid memory permission: {perms}")
        # check if new entry
        if self.get_map(address, address + size):
            raise XwInvalidMemoryAddress(f"Already mapped memory address: 0x{address:08X} (0x{size:08X})")
        # do map and add entry
        logger.debug(f"map_with_callback: 0x{address:08X} (0x{size:08X}) [{desc}]")
        # logger.debug(
        #     f"map_with_callback: {read_cb}, {user_data_read}, {write_cb}, {user_data_write}"
        # )
        self._box.mmio_map(address, size, read_cb, user_data_read, write_cb, user_data_write)
        self._registry.append(MemoryInfo(address, address + size, perms, buffer, desc or "Mapped_w_cb"))

    def _remap_read_callback(self, box: Uc, offset: int, size: int, user_data: Tuple) -> int:
        source, target = user_data
        data = int.from_bytes(box.mem_read(source + offset, size), self._endian_str)
        # logger.debug(f"Remap (R): 0x{target + offset:08x} ({size}), 0x{data:08x}")
        return data

    def _remap_write_callback(self, box: Uc, offset: int, size: int, data: int, user_data: Tuple) -> None:
        source, target = user_data
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid memory IO size to write: 0x{target + offset:08X} ({size})")
        # logger.debug(f"Remap (W): 0x{target + offset:08x} ({size}), 0x{data:08x}")
        value = Struct(f"<{self.size2fmt[size]}").pack(data)
        return box.mem_write(source + offset, value)

    def remap(
        self,
        source: int,
        target: int,
        size: Optional[int] = 0,
        perms: Optional[int] = UC_PROT_ALL,
        desc: Optional[str] = None,
    ) -> None:
        records = self.get_map(source, source + size)
        if not records:
            raise XwInvalidMemoryAddress(f"Unmapped memory address: 0x{source:08X} (0x{size:08X})")
        if source != records[0].start:
            raise XwInvalidMemoryAddress(f"Remap address mismatch: 0x{source:08X}, 0x{records[0].start:08X}")
        size_ = records[0].end - records[0].start
        if size <= 0:
            size = size_
        elif size != size_:
            raise XwInvalidMemorySize(f"Remap size mismatch: 0x{size:08X}, 0x{size_:08X}")
        self.map_with_callback(
            target,
            size,
            None,
            self._remap_read_callback,
            (source, target),
            self._remap_write_callback,
            (source, target),
            records[0].perms,
            f"{records[0].desc} (Remap)",
        )

    def _io_read_callback(self, box: Uc, offset: int, size: int, user_data: Tuple) -> int:
        base, buffer = user_data
        # logger.debug(f"IO read: 0x{base + offset:08x} ({size})")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid memory IO size to read: 0x{base + offset:08X} ({size})")
        records = self.get_io(base + offset, base + offset + size)
        if len(records) > 1:
            logger.debug(f"records: {records}")
            raise XwInvalidMemoryAddress(f"Invalid memory IO address to read: 0x{base + offset:08X} ({size})")
        if not records:
            logger.warning(f"Unmapped memory IO address to read: 0x{base + offset:08X} ({size})")
            return int.from_bytes(buffer[offset : offset + size], self._endian_str)
        else:
            return records[0].read_fn(base + offset, size)

    def _io_write_callback(self, box: Uc, offset: int, size: int, data: int, user_data: Tuple) -> None:
        base, buffer = user_data
        # logger.debug(f"IO write: 0x{base + offset:08x} ({size})")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid memory IO size to write: 0x{base + offset:08X} ({size})")
        records = self.get_io(base + offset, base + offset + size)
        if len(records) > 1:
            raise XwInvalidMemoryAddress(f"Invalid memory IO address to write: 0x{base + offset:08X} ({size})")
        if not records:
            logger.warning(f"Unmapped memory IO address to write: 0x{base + offset:08X} ({size})")
            value = Struct(f"<{self.size2fmt[size]}").pack(data)
            buffer[:] = buffer[:offset] + value + buffer[offset + size :]
        else:
            return records[0].write_fn(base + offset, size, data)

    def _bitband_read_callback(
        self,
        box: Uc,
        offset: int,
        size: int,
        user_data: Tuple,
        internal: Optional[bool] = False,
    ) -> int:
        base, buffer = user_data
        # logger.debug(f"Bit<= read: 0x{base + offset:08X} ({size})")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid bitband (R): 0x{base + offset:08X} ({size})")
        if buffer:
            records = self.get_io(base + offset, base + offset + size)
        else:
            records = self.get_map(base + offset, base + offset + size, True)
        if len(records) > 1:
            raise XwInvalidMemoryAddress(f"Invalid bitband (R): 0x{base + offset:08X} ({size})")
        if not records:
            data = int.from_bytes(buffer[offset : offset + size], self._endian_str)
            logger.warning(
                f"Unmapped bitband (R): 0x{base + offset:08X}{f' ({size})' if size != 4 else ''} => 0x{data:08x}"
            )
            return data
        if buffer:
            return records[0].read_fn(base + offset, size, internal)
        else:
            return int.from_bytes(box.mem_read(base + offset, size), self._endian_str)

    def _bitband_write_callback(
        self,
        box: Uc,
        offset: int,
        size: int,
        data: int,
        user_data: Tuple,
        internal: Optional[bool] = False,
    ) -> None:
        base, buffer = user_data
        # logger.debug(f"Bit<= write: 0x{base + offset:08x} ({size})")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid bitband size to write: 0x{base + offset:08X} ({size})")
        if buffer:
            records = self.get_io(base + offset, base + offset + size)
        else:
            records = self.get_map(base + offset, base + offset + size, True)
        if len(records) > 1:
            raise XwInvalidMemoryAddress(f"Invalid bitband address to write: 0x{base + offset:08X} ({size})")
        if not records:
            logger.warning(
                f"Unmapped bitband (W): 0x{base + offset:08X}{f' ({size})' if size != 4 else ''} <= 0x{data:08x}"
            )
            value = Struct(f"<{self.size2fmt[size]}").pack(data)
            buffer[:] = buffer[:offset] + value + buffer[offset + size :]
            return
        if buffer:
            return records[0].write_fn(base + offset, size, data, internal)
        else:
            value = Struct(f"<{self.size2fmt[size]}").pack(data)
            return box.mem_write(base + offset, value)

    def _bitband_alias_read_callback(self, box: Uc, offset: int, size: int, user_data: Tuple) -> int:
        base, alias_base, buffer = user_data
        # logger.debug(f"Bitband (R): 0x{alias_base + offset:08x} ({size})")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid bitband alias size to read: 0x{alias_base + offset:08X} ({size})")
        base_offset = offset >> (6 if self._is_64bit else 5)
        byte_num = base_offset & (0x7 if self._is_64bit else 0x3)
        base_offset &= ~(0x7 if self._is_64bit else 0x3)
        bit_num = (offset & (0x3F if self._is_64bit else 0x1F)) >> (3 if self._is_64bit else 2)
        # value = int.from_bytes(buffer[base_offset: base_offset + (8 if self._is_64bit else 4)], self._endian_str)
        value = self._bitband_read_callback(
            box,
            base_offset,
            (8 if self._is_64bit else 4),
            (base, buffer),
        )
        bit_value = value & (1 << (byte_num * 8 + bit_num))
        logger.debug(
            f"Bitband (R): 0x{alias_base + offset:08x} => 0x{base + base_offset + byte_num:08x}.{bit_num} => {1 if bit_value else 0}"
        )
        return 1 if bit_value else 0

    def _bitband_alias_write_callback(self, box: Uc, offset: int, size: int, data: int, user_data: Tuple) -> None:
        base, alias_base, buffer = user_data
        # logger.debug(f"Bitband (W0): 0x{alias_base + offset:08x} ({size}) <= 0x{data:x}")
        if size not in self.size2fmt:
            raise XwInvalidMemorySize(f"Invalid bitband alias size to write: 0x{alias_base + offset:08X} ({size})")
        base_offset = offset >> (6 if self._is_64bit else 5)
        byte_num = base_offset & (0x7 if self._is_64bit else 0x3)
        base_offset &= ~(0x7 if self._is_64bit else 0x3)
        bit_num = (offset & (0x3F if self._is_64bit else 0x1F)) >> (3 if self._is_64bit else 2)
        # value = int.from_bytes(buffer[base_offset: base_offset + (8 if self._is_64bit else 4)], self._endian_str)
        value = self._bitband_read_callback(box, base_offset, (8 if self._is_64bit else 4), (base, buffer), True)
        new_value = (value | (1 << (byte_num * 8 + bit_num))) if data else (value & ~(1 << (byte_num * 8 + bit_num)))
        # value_ = Struct(f"<{'Q' if self._is_64bit else 'I'}").pack(new_value)
        # buffer = buffer[:base_offset] + value_ + buffer[base_offset + (8 if self._is_64bit else 4):]
        logger.debug(
            f"Bitband (W): 0x{alias_base + offset:08x} => 0x{base + base_offset + byte_num:08x}.{bit_num} <= {1 if data else 0}"
        )
        return self._bitband_write_callback(
            box,
            base_offset,
            (8 if self._is_64bit else 4),
            new_value,
            (base, buffer),
        )

    def map_memory(self, chip: Dict[str, Any]) -> None:
        for device in chip.get("peripherals", []):
            name = list(device.keys())[0]
            if device[name]["type"] == "memory":
                base = device[name]["base"]
                size = device[name]["size"]
                self.map(base, size, desc=name)
            elif device[name]["type"] == "remap":
                base = device[name]["base"]
                size = device[name]["size"]
                alias_base = device[name]["alias"]
                self.remap(base, alias_base, size)
            elif device[name]["type"] == "bitband_memory":
                base = device[name]["base"]
                size = device[name]["size"]
                alias_base = device[name]["alias"]
                alias_size = size * (64 if self._is_64bit else 32)
                self.map_with_callback(
                    alias_base,
                    alias_size,
                    None,
                    self._bitband_alias_read_callback,
                    (base, alias_base, False),
                    self._bitband_alias_write_callback,
                    (base, alias_base, False),
                    UC_PROT_READ | UC_PROT_WRITE,
                    f"{name} (=> 0x{base:08x})",
                )
            elif device[name]["type"] == "peripheral":
                base = device[name]["base"]
                size = device[name]["size"]
                # TODO: remove buffer
                buffer = bytearray(Struct(f"<{size}B").pack(*([0] * size)))
                self.map_with_callback(
                    base,
                    size,
                    buffer,
                    self._io_read_callback,
                    (base, buffer),
                    self._io_write_callback,
                    (base, buffer),
                    UC_PROT_READ | UC_PROT_WRITE,
                    f"{name} (IO)",
                )
            elif device[name]["type"] == "bitband_peripheral":
                base = device[name]["base"]
                size = device[name]["size"]
                alias_base = device[name]["alias"]
                alias_size = size * (64 if self._is_64bit else 32)
                # TODO: remove buffer
                buffer = bytearray(Struct(f"<{size}B").pack(*([0] * size)))
                self.map_with_callback(
                    base,
                    size,
                    buffer,
                    self._bitband_read_callback,
                    (base, buffer),
                    self._bitband_write_callback,
                    (base, buffer),
                    UC_PROT_READ | UC_PROT_WRITE,
                    f"{name} (<= 0x{alias_base:08x})",
                )
                self.map_with_callback(
                    alias_base,
                    alias_size,
                    None,
                    self._bitband_alias_read_callback,
                    (base, alias_base, buffer),
                    self._bitband_alias_write_callback,
                    (base, alias_base, buffer),
                    UC_PROT_READ | UC_PROT_WRITE,
                    f"{name} (=> 0x{base:08x})",
                )

    def write(self, address: int, data: bytes) -> None:
        return self._box.mem_write(address, data)

    def read(self, address: int, size: int) -> bytearray:
        return self._box.mem_read(address, size)

    def read_word(self, address: int) -> int:
        from struct import unpack, calcsize

        return unpack(f"{self._endian}{self._word}", self.read(address, calcsize(self._word)))[0]
