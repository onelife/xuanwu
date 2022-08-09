# -*- coding: utf-8 -*-

from os import path
from typing import Optional, Any

from .config import logger
from .memory import MemoryController
from .exception import XwInvalidParameter, XwInvalidCodeFile

__all__ = ["ProgramLoader"]


class ElfFileReader(object):
    """Executable and linkable format"""

    @staticmethod
    def load(memory: MemoryController, code: str) -> int:
        from elftools.elf.elffile import ELFFile

        with open(code, "rb") as file:
            elf = ELFFile(file)
            for segment in elf.iter_segments():
                if segment["p_type"] != "PT_LOAD" or segment["p_filesz"] == 0:
                    continue
                memory.write(segment["p_paddr"], segment.data()[:segment["p_filesz"]])


class HexFileReader(object):
    """Intel hex format

    : 03 0030 00 02337A 1E
    [Start code] [Byte count] [Address] [Record type] [Data] [Checksum]

    Record type
    - 0, data
    - 1, end of file
    - 2, extended segment address
    - 3, start segment address
    - 4, extended linear address
    - 5, start linear address
    """

    @staticmethod
    def load(memory: MemoryController, code: str) -> int:
        from struct import unpack

        entry = 0
        with open(code, "r") as file:
            upper = 0
            while True:
                # read the line
                line = file.readline()
                if not line:
                    break
                # process the line
                count = int(line[1:3], 16)
                addr = unpack(">H", bytes.fromhex(line[3:7]))[0]
                type_ = int(line[7:9], 16)
                data = bytes.fromhex(line[9 : 9 + 2 * count])
                # validate checksum
                check = sum([int(line[i : i + 2], 16) for i in range(1, 9 + 2 * count + 2, 2)]) & 0xFF
                if check:
                    raise XwInvalidCodeFile(f"Invalid checksum: {line}")
                # the upper 16 bits of the 32-bit absolute address
                if type_ == 4:
                    upper = unpack(">H", data)[0]
                    # logger.debug(f"upper: 0x{upper << 16:08x}")
                # the program code
                elif type_ == 0:
                    memory.write((upper << 16) + addr, data)
                # the 32-bit absolute address of program entry point
                elif type_ == 5:
                    entry = unpack(">I", data)[0]
                # end
                elif type_ == 1:
                    break
        return entry


class BinFileReader(object):
    """Binary format"""

    def __init__(self, load_address: int, **kwargs: Any):
        super().__init__(**kwargs)
        self._load_address = load_address

    def load(self, memory: MemoryController, code: str) -> int:
        with open(code, "rb") as file:
            memory.write(self._load_address, file.read())
        return 0x4


class ProgramLoader(object):
    FORMAT = {
        "e_ident": "16B",
        "HDR1": "{endian}2H5L6H",
        "SEC_HDR": "{endian}10L",
        "PGM_HDR": "{endian}8L",
    }

    def __init__(self, code: str, load_address: Optional[int] = None, **kwargs: Any):
        super().__init__(**kwargs)
        if not path.exists(code):
            raise XwInvalidParameter("Invalid code path: {code}")
        _, ext = path.splitext(code)
        self._code = code
        if ext == ".elf":
            self._loader = ElfFileReader()
        elif ext == ".hex":
            self._loader = HexFileReader()
        elif ext == ".bin":
            if load_address is None:
                raise XwInvalidParameter("No loading address provided")
            # logger.debug(f"load_address: 0x{load_address:08x}")
            self._loader = BinFileReader(load_address)
        else:
            raise XwInvalidParameter("Unsupported file extension: {ext}")

    def load(self, memory: MemoryController):
        return self._loader.load(memory, self._code)


if __name__ == "__main__":
    loader = ProgramLoader("/home/onelife/workspace/qiling/examples/rootfs/mcu/stm32f411/Blink.ino.elf")
    # loader.load()
