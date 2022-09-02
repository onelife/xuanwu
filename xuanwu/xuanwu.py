# -*- coding: utf-8 -*-

from os import path
from typing import Optional, Union, Any

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
# from unicorn import UC_HOOK_INTR, UC_HOOK_CODE, UC_HOOK_BLOCK
from capstone import Cs, CS_ARCH_ARM, CS_MODE_ARM, CS_MODE_MCLASS, CS_MODE_THUMB

# from keystone import Ks, KS_ARCH_ARM, KS_MODE_ARM, KS_MODE_THUMB
# from unicorn import UC_PROT_NONE, UC_PROT_READ, UC_PROT_WRITE, UC_PROT_EXEC, UC_PROT_ALL
import yaml

from .register import RegisterController
from .memory import MemoryController
from .loader import ProgramLoader
from .rsp import RemoteSerialProtocol
from .exception import XwInvalidParameter, XwInvalidChipInformation


__all__ = ["XuanWu"]

ARCH_MAPPING = {
    "arm": (UC_ARCH_ARM, CS_ARCH_ARM),
}

MODE_MAPPING = {
    "arm": {
        "cortex_m": (
            UC_MODE_ARM + UC_MODE_MCLASS + UC_MODE_THUMB,
            CS_MODE_ARM + CS_MODE_MCLASS + CS_MODE_THUMB,
        ),
    },
}

BIG_ENDIAN = {}

ADDRESS_64BIT = {}


# def hook_block(uc, address, size, user_data):
#     print(">>> Tracing basic block at 0x%x, block size = 0x%x" %(address, size))

# def hook_code(uc, address, size, user_data):
#     print(">>> Tracing code at 0x%x, block size = 0x%x" %(address, size))

# def hook_int(uc, address, size, user_data):
#     print(">>> Tracing interrupt at 0x%x, block size = 0x%x" %(address, size))


class XuanWu(object):
    """xxx"""

    def __init__(self, chip: str, code: str, rsp: Union[bool, int] = False, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # load chip info
        if not path.exists(chip):
            raise XwInvalidParameter("Invalid chip information path: {chip}")
        with open(chip) as file:
            doc = yaml.safe_load(file)
        if "chip" not in doc:
            raise XwInvalidChipInformation("Invalid chip information file: {chip}")
        self._chip = doc["chip"]
        # create virtual box and disassembler
        arch = self._chip.get("arch")
        mode = self._chip.get("mode")
        self._arch = ARCH_MAPPING.get(arch)
        self._mode = MODE_MAPPING.get(arch, {}).get(mode)
        if not all([self._arch, self._mode]):
            raise XwInvalidParameter(f"Unknown architecture or mode: {arch}, {mode}")
        self.box = Uc(self._arch[0], self._mode[0])
        # self.box.hook_add(UC_HOOK_BLOCK, hook_block)
        # self.box.hook_add(UC_HOOK_CODE, hook_code)
        # self.box.hook_add(UC_HOOK_INTR, hook_int)
        self.dasm = Cs(self._arch[1], self._mode[1])
        self.dasm.detail = True
        # endian and address size
        # TODO: UC_MODE_LITTLE_ENDIAN ?
        self.endian = "big" if self._chip.get("arch") in BIG_ENDIAN else "little"
        self.is_64bit = self._chip.get("arch") in ADDRESS_64BIT
        # create register controller
        self.reg = RegisterController(self.box, self._arch[0], self._mode[0])
        # create memory controller
        self.mem = MemoryController(self.box, self.endian == "little", self.is_64bit)
        # create hardware controller
        self.hw = None
        if self._arch[0] == UC_ARCH_ARM:
            from .arch import ArmHardwareController

            self.hw = ArmHardwareController(self.box, self.reg, self.mem)
        if self.hw is None:
            raise XwInvalidParameter(f"Unknown architecture {arch}")
        # map memory
        self.mem.map_memory(self._chip)
        # map memory for peripherals
        self.hw.map_memory(self._chip)
        # create program loader
        if not path.exists(code):
            raise XwInvalidParameter("Invalid code path: {code}")
        self._loader = ProgramLoader(code, 0)
        self._loader.load(self.mem)
        # gdb rsp
        if rsp:
            if isinstance(rsp, bool):
                self.rsp = RemoteSerialProtocol(self.box, self.mem, self.reg, self._arch[0], self._mode[0])
            else:
                self.rsp = RemoteSerialProtocol(self.box, self.mem, self.reg, self._arch[0], self._mode[0], rsp)
        else:
            self.rsp = None

    def show_inst(self, start_offset: int, end_offset: int) -> None:
        start = self.reg.pc + 4 * start_offset
        count = max(1, end_offset - start_offset + 1)
        data = self.mem.read(start, 4 * count)
        offset = 0
        info = []
        pc_idx = 0
        # print(f"pc: 0x{self.reg.pc:08x}")
        # print(f"r0: 0x{self.reg.r0:08x}")
        # print(f"r1: 0x{self.reg.r1:08x}")
        # print(f"r2: 0x{self.reg.r2:08x}")
        # print(f"r3: 0x{self.reg.r3:08x}")
        # print(f"r4: 0x{self.reg.r4:08x}")
        # print(f"r5: 0x{self.reg.r5:08x}")
        while offset < 4 * count:
            length = 2 if self.reg._t_mode else 4
            address_ = start + offset
            data_ = data[offset : offset + length]
            # disassemble
            try:
                inst = next(self.dasm.disasm(data_, address_))
                # dasm_ = f"{inst.mnemonic} {inst.op_str}"
            except StopIteration:
                if length == 2:
                    length = 4
                    data_ = data[offset : offset + length]
                    try:
                        inst = next(self.dasm.disasm(data_, address_))
                        # dasm_ = f"{inst.mnemonic} {inst.op_str}"
                    except StopIteration:
                        inst = None
                        # dasm_ = None
                else:
                    inst = None
                    # dasm_ = None
            offset += length
            if address_ == self.reg.pc:
                pc_idx = len(info)
            info.append((address_, data_, inst))
        # print message
        start_idx = max(0, pc_idx + start_offset)
        end_idx = min(len(info) - 1, pc_idx + end_offset)
        for address_, data_, inst in info[start_idx : end_idx + 1]:
            message = f"{address_:08x} ({int.from_bytes(data_, self.endian):08x})"
            if inst:
                message += f": {inst.mnemonic} {inst.op_str}"
            print(message)
            read_regs, modify_regs = inst.regs_access()
            for reg in read_regs:
                print(f"[R] {inst.reg_name(reg)} = 0x{self.reg.read(reg):08x}")
            for reg in modify_regs:
                print(f"[M] {inst.reg_name(reg)} = 0x{self.reg.read(reg):08x}")

    def run(self, until: Optional[int] = 0x0, count: Optional[int] = 0):
        from unicorn import UcError, UC_ERR_READ_UNMAPPED, UC_ERR_WRITE_UNMAPPED

        try:
            if not self.rsp:
                self.box.emu_start(self.reg.pc_t, until, count)
            else:
                self.rsp.run()
        except UcError as err:
            if err.errno in (UC_ERR_READ_UNMAPPED, UC_ERR_WRITE_UNMAPPED):
                self.show_inst(-1, 1)
            raise

    def reset(self) -> None:
        self.hw.reset()
        # TODO: B1.5.5
        self.reg.write("lr", 0xFFFFFFFF)
        # select and initialize msp
        self.reg.write("msp", self.mem.read_word(0x0))
        self.reg.write("pc", self.mem.read_word(0x4))
